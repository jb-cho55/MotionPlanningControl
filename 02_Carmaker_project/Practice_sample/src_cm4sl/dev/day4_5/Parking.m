function [desired_ax, steer_fl, steer_fr, path_x_dbg, path_y_dbg, path_len_dbg, selector_ctrl] = Parking(ego_x, ego_y, ego_yaw, ego_v, start_point, finish_point, goal_yaw, occ_map)
%PARKING  Self-contained planner + controllers for the Day4_5_Scenario_1.slx
%   "MATLAB Function" (Parking) block.
%
%   Hybrid A* (with Reeds-Shepp analytic-shot endgame) -> Stanley + PD.
%
%   Inputs
%       ego_x, ego_y, ego_yaw, ego_v : ego rear-bumper pose (m, rad) + speed.
%       start_point                  : 1x3 (unused — kept for inport wiring).
%       finish_point                 : 1x3 [x y *] T00 rear-bumper goal.
%       goal_yaw                     : T00 heading (rad; deg auto-converted).
%       occ_map                      : 200x200 occupancy grid from add_obstacle_.
%
%   Outputs
%       desired_ax       : longitudinal accel command (m/s^2).
%       steer_fl/fr      : front-wheel angle (rad), saturated.
%       path_x_dbg       : 300x1 path buffer (for monitoring).
%       path_y_dbg       : 300x1 path buffer.
%       path_len_dbg     : int32 — number of valid path samples.
%       selector_ctrl    : EV6 DM.SelectorCtrl (+1 drive, -1 reverse).
%
%   This file mirrors the chart script verbatim.  All helpers
%   (hybrid_astar_plan, stanley, pd_speed, rs_shot, compute_grid_heuristic,
%   compute_clearance, map_const) are inlined as local functions so the .slx
%   needs no external .m files.
%
%#codegen

MAX_PATH = int32(300);
REPLAN_PERIOD = int32(3000);

% path_dir stored as double (+1/-1) to avoid Stateflow int8 inference issues
persistent path_x path_y path_yaw path_dir path_len tick last_gx last_gy init
if isempty(init)
    path_x   = zeros(MAX_PATH, 1);
    path_y   = zeros(MAX_PATH, 1);
    path_yaw = zeros(MAX_PATH, 1);
    path_dir = ones(MAX_PATH, 1);
    path_len = int32(0);
    tick = int32(REPLAN_PERIOD + 1);
    last_gx = 1.0e9;
    last_gy = 1.0e9;
    init = true;
end

u_unused = start_point(1) * 0;   %#ok<NASGU>

t00_x = finish_point(1);
t00_y = finish_point(2);
t00_yaw = goal_yaw;
if abs(ego_yaw) > 2.0 * pi; ego_yaw = ego_yaw * pi / 180.0; end
if abs(t00_yaw) > 2.0 * pi; t00_yaw = t00_yaw * pi / 180.0; end

need_replan = false;
if path_len < int32(2); need_replan = true; end
if abs(t00_x - last_gx) > 0.2 || abs(t00_y - last_gy) > 0.2; need_replan = true; end
if tick >= REPLAN_PERIOD; need_replan = true; end

if need_replan
    [px, py, pyaw, pdir, plen] = hybrid_astar_plan(ego_x, ego_y, ego_yaw, t00_x, t00_y, t00_yaw, uint8(occ_map));
    if plen >= int32(2)
        for i = int32(1):MAX_PATH
            path_x(i)   = px(i);
            path_y(i)   = py(i);
            path_yaw(i) = pyaw(i);
            path_dir(i) = double(pdir(i));
        end
        path_len = plen;
        tick = int32(0);
        last_gx = t00_x;
        last_gy = t00_y;
    elseif path_len < int32(2)
        path_x(:) = 0.0; path_y(:) = 0.0; path_yaw(:) = 0.0;
        path_dir(:) = 1.0;
        path_x(1) = ego_x; path_y(1) = ego_y; path_yaw(1) = ego_yaw;
        path_x(2) = ego_x; path_y(2) = ego_y; path_yaw(2) = ego_yaw;
        path_len = int32(2);
        tick = int32(0);
    end
end
tick = tick + int32(1);

stay_put = path_len <= int32(2) && hypot(path_x(2) - path_x(1), path_y(2) - path_y(1)) < 0.01;

d_goal = hypot(t00_x - ego_x, t00_y - ego_y);
if stay_put
    v_des = 0.0;
elseif d_goal > 15.0
    v_des = 3.0;
elseif d_goal > 6.0
    v_des = 1.6;
elseif d_goal > 1.5
    v_des = 0.8;
elseif d_goal > 0.4
    v_des = 0.3;
else
    v_des = 0.0;
end

% Convert path_dir (double) -> int8 buffer for stanley signature.
pdir_i8 = int8(zeros(MAX_PATH, 1));
for i = int32(1):MAX_PATH
    if path_dir(i) < 0
        pdir_i8(i) = int8(-1);
    else
        pdir_i8(i) = int8(1);
    end
end

[steer_cmd, dir_sign] = stanley(ego_x, ego_y, ego_yaw, ego_v, path_x, path_y, path_yaw, pdir_i8, path_len, t00_yaw);

% Map dir_sign to gear command (EV6: DM.SelectorCtrl, -1 = reverse, 1 = drive).
selector_ctrl = 1.0;
if dir_sign == int8(-1)
    selector_ctrl = -1.0;
    v_des = -abs(v_des);
end
if stay_put
    steer_cmd = 0.0;
    selector_ctrl = 1.0;
end
desired_ax = pd_speed(v_des, ego_v);

steer_fl = steer_cmd;
steer_fr = steer_cmd;

path_x_dbg = path_x;
path_y_dbg = path_y;
path_len_dbg = path_len;
end

%% =====================================================================
%% Local helpers — Hybrid A* planner
%% =====================================================================

function [path_x, path_y, path_yaw, path_dir, path_len] = hybrid_astar_plan(sx, sy, syaw, gx, gy, gyaw, occ_map)
%#codegen
MAX_NODES = int32(20000);
MAX_PATH  = int32(300);
N_STEER   = int32(7);
N_ACTION  = int32(14);

WHEELBASE   = 2.8;
STEP_DIST   = 1.0;
STEERS = [-0.50, -0.32, -0.16, 0.0, 0.16, 0.32, 0.50];
SWITCH_PENALTY = 4.0;
POS_RES = 0.5;
YAW_RES = pi / 12.0;
W_HEUR   = 1.3;
W_YAW    = 1.0;

h_grid    = compute_grid_heuristic(occ_map, gx, gy);
clear_map = compute_clearance(occ_map);

shared = map_const_local();
CLEAR_MAX = shared.CLEAR_MAX;
W_CLEAR   = shared.W_CLEAR;
BOX_L     = shared.PARK_BOX_L;
BOX_W     = shared.PARK_BOX_W;
BOX_TOL   = shared.PARK_TOL;
EGO_L     = shared.EGO_L;
EGO_W     = shared.EGO_W;

path_x   = zeros(1, MAX_PATH);
path_y   = zeros(1, MAX_PATH);
path_yaw = zeros(1, MAX_PATH);
path_dir = zeros(1, MAX_PATH, 'int8');
path_len = int32(0);

nx  = zeros(MAX_NODES, 1);
ny  = zeros(MAX_NODES, 1);
nyaw = zeros(MAX_NODES, 1);
ng  = zeros(MAX_NODES, 1);
nf  = zeros(MAX_NODES, 1);
nparent = zeros(MAX_NODES, 1, 'int32');
ndir    = zeros(MAX_NODES, 1, 'int8');
nopen   = false(MAX_NODES, 1);
nclosed = false(MAX_NODES, 1);

kx = zeros(MAX_NODES, 1, 'int32');
ky = zeros(MAX_NODES, 1, 'int32');
kw = zeros(MAX_NODES, 1, 'int32');

node_count = int32(1);
sy_w = wrap_pi(syaw);
nx(1)   = sx;
ny(1)   = sy;
nyaw(1) = sy_w;
ng(1)   = 0.0;
h0 = lookup_h(sx, sy, h_grid) + W_YAW * abs(angle_diff(gyaw, sy_w));
nf(1)   = W_HEUR * h0;
nparent(1) = int32(0);
ndir(1)    = int8(1);
nopen(1)   = true;
kx(1) = int32(round(sx / POS_RES));
ky(1) = int32(round(sy / POS_RES));
kw(1) = int32(round(wrap_2pi(sy_w) / YAW_RES));

goal_idx = int32(0);
rs_attached = false;
rs_px   = zeros(1, MAX_PATH);
rs_py   = zeros(1, MAX_PATH);
rs_pyaw = zeros(1, MAX_PATH);
rs_pdir = zeros(1, MAX_PATH, 'int8');
rs_plen = int32(0);

for iter = 1:MAX_NODES
    cur = int32(0);
    best_f = 1.0e12;
    for i = int32(1):node_count
        if nopen(i) && nf(i) < best_f
            best_f = nf(i);
            cur = i;
        end
    end
    if cur == int32(0)
        break;
    end

    nopen(cur)   = false;
    nclosed(cur) = true;

    cx = nx(cur); cy = ny(cur); cyaw = nyaw(cur);

    if pose_in_goal_box(cx, cy, cyaw, gx, gy, gyaw, ...
                        BOX_L, BOX_W, BOX_TOL, EGO_L, EGO_W)
        goal_idx = cur;
        break;
    end

    d_to_goal = hypot(gx - cx, gy - cy);
    if d_to_goal < 15.0
        R = WHEELBASE / tan(0.5);
        [rs_px, rs_py, rs_pyaw, rs_pdir, rs_plen, rs_ok] = ...
            rs_shot(cx, cy, cyaw, gx, gy, gyaw, occ_map, R, STEP_DIST);
        if rs_ok && rs_plen >= int32(2)
            rs_len = 0.0;
            for k = int32(2):rs_plen
                rs_len = rs_len + hypot(rs_px(k) - rs_px(k-1), rs_py(k) - rs_py(k-1));
            end
            if rs_len < 2.0 * max(d_to_goal, 1.0)
                goal_idx = cur;
                rs_attached = true;
                break;
            end
        end
    end

    for a = int32(1):N_ACTION
        if a <= N_STEER
            delta = STEERS(a);
            ds    = STEP_DIST;
            dir_a = int8(1);
        else
            delta = STEERS(a - N_STEER);
            ds    = -STEP_DIST;
            dir_a = int8(-1);
        end

        [nx_n, ny_n, nyaw_n] = bicycle_step(cx, cy, cyaw, delta, ds, WHEELBASE);

        in_start_bubble = (hypot(cx - sx, cy - sy) < 5.0);
        if ~in_start_bubble && is_collision_segment(cx, cy, nx_n, ny_n, occ_map)
            continue;
        end

        kxn = int32(round(nx_n / POS_RES));
        kyn = int32(round(ny_n / POS_RES));
        kwn = int32(round(wrap_2pi(nyaw_n) / YAW_RES));

        dup_idx = int32(0);
        for j = int32(1):node_count
            if kx(j) == kxn && ky(j) == kyn && kw(j) == kwn
                dup_idx = j;
                break;
            end
        end

        step_cost = STEP_DIST * (1.0 + 0.2 * abs(delta));
        if dir_a ~= ndir(cur)
            step_cost = step_cost + SWITCH_PENALTY;
        end
        clr = lookup_h(nx_n, ny_n, clear_map);
        if clr < CLEAR_MAX
            step_cost = step_cost + W_CLEAR * (CLEAR_MAX - clr);
        end
        new_g = ng(cur) + step_cost;
        h_grid_n = lookup_h(nx_n, ny_n, h_grid);
        if h_grid_n >= 1.0e9
            h_grid_n = hypot(gx - nx_n, gy - ny_n);
        end
        h = h_grid_n + W_YAW * abs(angle_diff(gyaw, nyaw_n));
        new_f = new_g + W_HEUR * h;

        if dup_idx ~= int32(0)
            if nclosed(dup_idx); continue; end
            if new_g >= ng(dup_idx); continue; end
            nx(dup_idx)  = nx_n;
            ny(dup_idx)  = ny_n;
            nyaw(dup_idx) = nyaw_n;
            ng(dup_idx)  = new_g;
            nf(dup_idx)  = new_f;
            nparent(dup_idx) = cur;
            ndir(dup_idx)    = dir_a;
            nopen(dup_idx)   = true;
        else
            if node_count >= MAX_NODES
                break;
            end
            node_count = node_count + int32(1);
            nx(node_count)   = nx_n;
            ny(node_count)   = ny_n;
            nyaw(node_count) = nyaw_n;
            ng(node_count)   = new_g;
            nf(node_count)   = new_f;
            nparent(node_count) = cur;
            ndir(node_count)    = dir_a;
            nopen(node_count)   = true;
            kx(node_count) = kxn;
            ky(node_count) = kyn;
            kw(node_count) = kwn;
        end
    end
end

if goal_idx == int32(0)
    return;
end

tmp_x   = zeros(MAX_PATH, 1);
tmp_y   = zeros(MAX_PATH, 1);
tmp_yaw = zeros(MAX_PATH, 1);
tmp_dir = zeros(MAX_PATH, 1, 'int8');
cnt = int32(0);
idx = goal_idx;
while idx > int32(0) && cnt < MAX_PATH
    cnt = cnt + int32(1);
    tmp_x(cnt)   = nx(idx);
    tmp_y(cnt)   = ny(idx);
    tmp_yaw(cnt) = nyaw(idx);
    tmp_dir(cnt) = ndir(idx);
    idx = nparent(idx);
end

for k = int32(1):cnt
    src = cnt - k + int32(1);
    path_x(k)   = tmp_x(src);
    path_y(k)   = tmp_y(src);
    path_yaw(k) = tmp_yaw(src);
    path_dir(k) = tmp_dir(src);
end
path_len = cnt;

if rs_attached && rs_plen >= int32(2)
    for k = int32(2):rs_plen
        if path_len >= MAX_PATH
            break;
        end
        path_len = path_len + int32(1);
        path_x(path_len)   = rs_px(k);
        path_y(path_len)   = rs_py(k);
        path_yaw(path_len) = rs_pyaw(k);
        path_dir(path_len) = rs_pdir(k);
    end
end
end

%% =====================================================================
%% Local helpers — Stanley lateral controller
%% =====================================================================

function [steer_cmd, dir_sign] = stanley(ego_x, ego_y, ego_yaw, ego_v, ...
                                          path_x, path_y, path_yaw, path_dir, ...
                                          path_len, goal_yaw)
%#codegen
c = map_const_local();
MAX_PATH    = int32(300);
MAX_STEER   = 0.5;
K_E         = 1.5;
V_SOFT      = 1.0;
END_RADIUS  = 3.0;

steer_cmd = 0.0;
dir_sign = int8(1);
if path_len < int32(2)
    return;
end

plen = int32(min(int32(path_len), MAX_PATH));

nearest_idx = int32(1);
best_d2 = 1.0e18;
for i = int32(1):plen
    dx = path_x(i) - ego_x;
    dy = path_y(i) - ego_y;
    d2 = dx*dx + dy*dy;
    if d2 < best_d2
        best_d2 = d2;
        nearest_idx = i;
    end
end

have_path_yaw = false;
if numel(path_yaw) >= double(plen)
    if any(abs(path_yaw(1:min(plen, int32(20)))) > 1.0e-6)
        have_path_yaw = true;
    end
end

if have_path_yaw
    path_heading = path_yaw(nearest_idx);
else
    if nearest_idx < plen
        path_heading = atan2(path_y(nearest_idx+1) - path_y(nearest_idx), ...
                             path_x(nearest_idx+1) - path_x(nearest_idx));
    elseif nearest_idx > 1
        path_heading = atan2(path_y(nearest_idx) - path_y(nearest_idx-1), ...
                             path_x(nearest_idx) - path_x(nearest_idx-1));
    else
        path_heading = goal_yaw;
    end
end

end_dist = 0.0;
prev_x = path_x(nearest_idx);
prev_y = path_y(nearest_idx);
for i = nearest_idx+int32(1):plen
    end_dist = end_dist + hypot(path_x(i) - prev_x, path_y(i) - prev_y);
    prev_x = path_x(i);
    prev_y = path_y(i);
end

if end_dist < END_RADIUS
    alpha = 1.0 - end_dist / END_RADIUS;
    if alpha < 0.0; alpha = 0.0; end
    if alpha > 1.0; alpha = 1.0; end
    diff = wrap_pi(goal_yaw - path_heading);
    path_heading = wrap_pi(path_heading + alpha * diff);
end

if numel(path_dir) >= double(nearest_idx)
    dir_sign = int8(sign_default(path_dir(nearest_idx), int8(1)));
end

eff_heading = path_heading;
if dir_sign == int8(-1)
    eff_heading = wrap_pi(path_heading + pi);
end
heading_err = wrap_pi(eff_heading - ego_yaw);

dx = ego_x - path_x(nearest_idx);
dy = ego_y - path_y(nearest_idx);
c_ph = cos(path_heading);
s_ph = sin(path_heading);
cross_track =  s_ph * dx - c_ph * dy;

cross_track_eff = cross_track;
if dir_sign == int8(-1)
    cross_track_eff = -cross_track;
end
steer_cmd = heading_err + atan2(K_E * cross_track_eff, V_SOFT + abs(ego_v));
if dir_sign == int8(-1)
    steer_cmd = -steer_cmd;
end

if steer_cmd > MAX_STEER
    steer_cmd = MAX_STEER;
elseif steer_cmd < -MAX_STEER
    steer_cmd = -MAX_STEER;
end

u_unused = c.WHEELBASE * 0.0;       %#ok<NASGU>
end

%% =====================================================================
%% Local helpers — PD longitudinal controller
%% =====================================================================

function desired_ax = pd_speed(v_des, v_ego)
%#codegen
DT = 0.01;
KP = 1.5;
KD = 0.3;
AX_MIN = -3.0;
AX_MAX = 1.5;
ALPHA  = 0.6;

persistent e_prev d_lpf init
if isempty(init)
    e_prev = 0.0;
    d_lpf  = 0.0;
    init   = true;
end

e   = v_des - v_ego;
e_d = (e - e_prev) / DT;
d_lpf = ALPHA * d_lpf + (1.0 - ALPHA) * e_d;
e_prev = e;

desired_ax = KP * e + KD * d_lpf;

if desired_ax > AX_MAX
    desired_ax = AX_MAX;
elseif desired_ax < AX_MIN
    desired_ax = AX_MIN;
end
end

%% =====================================================================
%% Local helpers — Reeds-Shepp analytic shot (CSC subset)
%% =====================================================================

function [px, py, pyaw, pdir, plen, ok] = rs_shot(sx, sy, syaw, gx, gy, gyaw, occ_map, R, ds)
%#codegen
MAX_PATH = int32(300);
px   = zeros(1, MAX_PATH);
py   = zeros(1, MAX_PATH);
pyaw = zeros(1, MAX_PATH);
pdir = zeros(1, MAX_PATH, 'int8');
plen = int32(0);
ok   = false;

patterns = [...
    +1, +1, +1;
    +1, +1, -1;
    -1, +1, +1;
    -1, +1, -1;
    +1, -1, +1;
    +1, -1, -1;
    -1, -1, +1;
    -1, -1, -1];

best_len = 1.0e18;
for p = 1:size(patterns, 1)
    t1 = patterns(p, 1);
    sd = patterns(p, 2);
    t2 = patterns(p, 3);
    [c_ok, c_px, c_py, c_pyaw, c_pdir, c_plen, c_len] = ...
        try_csc(sx, sy, syaw, gx, gy, gyaw, R, ds, t1, sd, t2, occ_map, MAX_PATH);
    if c_ok && c_len < best_len
        best_len = c_len;
        px   = c_px;
        py   = c_py;
        pyaw = c_pyaw;
        pdir = c_pdir;
        plen = c_plen;
        ok   = true;
    end
end
end

function [ok, px, py, pyaw, pdir, plen, total_len] = try_csc(sx, sy, syaw, ...
        gx, gy, gyaw, R, ds, t1, sd, t2, occ_map, MAX_PATH)
%#codegen
ok = false;
px = zeros(1, MAX_PATH); py = zeros(1, MAX_PATH);
pyaw = zeros(1, MAX_PATH); pdir = zeros(1, MAX_PATH, 'int8');
plen = int32(0);
total_len = 1.0e18;

c1x = sx - R * sin(syaw) * t1;
c1y = sy + R * cos(syaw) * t1;
c2x = gx - R * sin(gyaw) * t2;
c2y = gy + R * cos(gyaw) * t2;

dxc = c2x - c1x;
dyc = c2y - c1y;
d_cc = hypot(dxc, dyc);

if t1 == t2
    if d_cc < 1.0e-6
        return;
    end
    seg_len = d_cc;
    alpha = atan2(dyc, dxc);
    tp1x = c1x + R * sin(alpha) * t1;
    tp1y = c1y - R * cos(alpha) * t1;
    tp2x = tp1x + seg_len * cos(alpha);
    tp2y = tp1y + seg_len * sin(alpha);
    tangent_yaw = alpha;
else
    if d_cc < 2.0 * R
        return;
    end
    seg_len = sqrt(d_cc * d_cc - 4.0 * R * R);
    alpha = atan2(dyc, dxc);
    beta = atan2(2.0 * R, seg_len) * t1;
    tan_dir = alpha - beta;
    tp1x = c1x + R * sin(tan_dir) * t1;
    tp1y = c1y - R * cos(tan_dir) * t1;
    tp2x = tp1x + seg_len * cos(tan_dir);
    tp2y = tp1y + seg_len * sin(tan_dir);
    tangent_yaw = tan_dir;
end

arc1_yaw_start = wrap_pi(syaw);
arc1_yaw_end   = wrap_pi(tangent_yaw);
arc1_len_signed = arc_length_along_circle(arc1_yaw_start, arc1_yaw_end, t1) * R;

arc2_yaw_start = wrap_pi(tangent_yaw);
arc2_yaw_end   = wrap_pi(gyaw);
arc2_len_signed = arc_length_along_circle(arc2_yaw_start, arc2_yaw_end, t2) * R;

total_len = abs(arc1_len_signed) + seg_len + abs(arc2_len_signed);

n_arc1 = max(int32(1), int32(ceil(abs(arc1_len_signed) / ds)));
n_seg  = max(int32(1), int32(ceil(seg_len / ds)));
n_arc2 = max(int32(1), int32(ceil(abs(arc2_len_signed) / ds)));
total_n = n_arc1 + n_seg + n_arc2 + int32(1);
if total_n > MAX_PATH
    return;
end

idx = int32(1);
px(idx) = sx; py(idx) = sy; pyaw(idx) = syaw; pdir(idx) = int8(sd);

for k = int32(1):n_arc1
    s = double(k) / double(n_arc1);
    yawk = arc1_yaw_start + s * (arc1_len_signed / R);
    xk = c1x + R * sin(yawk) * t1;
    yk = c1y - R * cos(yawk) * t1;
    idx = idx + int32(1);
    px(idx) = xk; py(idx) = yk; pyaw(idx) = wrap_pi(yawk); pdir(idx) = int8(sd);
end

for k = int32(1):n_seg
    s = double(k) / double(n_seg);
    xk = tp1x + s * (tp2x - tp1x);
    yk = tp1y + s * (tp2y - tp1y);
    idx = idx + int32(1);
    px(idx) = xk; py(idx) = yk; pyaw(idx) = tangent_yaw; pdir(idx) = int8(sd);
end

for k = int32(1):n_arc2
    s = double(k) / double(n_arc2);
    yawk = arc2_yaw_start + s * (arc2_len_signed / R);
    xk = c2x + R * sin(yawk) * t2;
    yk = c2y - R * cos(yawk) * t2;
    idx = idx + int32(1);
    px(idx) = xk; py(idx) = yk; pyaw(idx) = wrap_pi(yawk); pdir(idx) = int8(sd);
end

plen = idx;

for i = int32(1):plen-int32(1)
    if is_collision_segment(px(i), py(i), px(i+1), py(i+1), occ_map)
        plen = int32(0);
        return;
    end
end

ok = true;
end

function len = arc_length_along_circle(yaw_start, yaw_end, turn_sign)
%#codegen
d = wrap_pi(yaw_end - yaw_start);
if turn_sign > 0
    if d < 0
        d = d + 2.0 * pi;
    end
else
    if d > 0
        d = d - 2.0 * pi;
    end
end
len = d;
end

%% =====================================================================
%% Local helpers — Heuristic + clearance grids
%% =====================================================================

function h_grid = compute_grid_heuristic(occ_map, gx, gy)
%#codegen
c = map_const_local();
N = double(c.N);
res = c.RES;
N_int = int32(N);

INF = single(1.0e9);
h_grid = INF * ones(N, N, 'single');

gc = floor((gx - c.X_MIN) / res) + 1;
gr = floor((c.Y_MAX - gy) / res) + 1;
if gc < 1 || gc > N || gr < 1 || gr > N
    return;
end
if occ_map(int32(gr), int32(gc)) > 0
    return;
end

MAX_Q = int32(8 * N * N);
qr = zeros(MAX_Q, 1, 'int32');
qc = zeros(MAX_Q, 1, 'int32');
qhead = int32(1);
qtail = int32(2);

h_grid(int32(gr), int32(gc)) = single(0);
qr(1) = int32(gr);
qc(1) = int32(gc);

DR = int32([-1 -1 -1  0  0  1  1  1]);
DC = int32([-1  0  1 -1  1 -1  0  1]);
SQ2 = single(sqrt(2.0));
COSTS = single([SQ2 1 SQ2 1 1 SQ2 1 SQ2]) * single(res);

while qhead < qtail
    r = qr(qhead);
    cc = qc(qhead);
    qhead = qhead + int32(1);

    h_here = h_grid(r, cc);

    for k = int32(1):int32(8)
        nr = r + DR(k);
        nc = cc + DC(k);
        if nr < 1 || nr > N_int || nc < 1 || nc > N_int
            continue;
        end
        if occ_map(nr, nc) > 0
            continue;
        end
        new_h = h_here + COSTS(k);
        if new_h < h_grid(nr, nc)
            h_grid(nr, nc) = new_h;
            if qtail <= MAX_Q
                qr(qtail) = nr;
                qc(qtail) = nc;
                qtail = qtail + int32(1);
            end
        end
    end
end
end

function clear_map = compute_clearance(occ_map)
%#codegen
c = map_const_local();
N = double(c.N);
res = c.RES;
N_int = int32(N);

INF = single(1.0e9);
clear_map = INF * ones(N, N, 'single');

MAX_Q = int32(8 * N * N);
qr = zeros(MAX_Q, 1, 'int32');
qc = zeros(MAX_Q, 1, 'int32');
qhead = int32(1);
qtail = int32(1);

for r = int32(1):N_int
    for cc = int32(1):N_int
        if occ_map(r, cc) > 0
            clear_map(r, cc) = single(0);
            qr(qtail) = r;
            qc(qtail) = cc;
            qtail = qtail + int32(1);
        end
    end
end

DR = int32([-1 -1 -1  0  0  1  1  1]);
DC = int32([-1  0  1 -1  1 -1  0  1]);
SQ2 = single(sqrt(2.0));
COSTS = single([SQ2 1 SQ2 1 1 SQ2 1 SQ2]) * single(res);

while qhead < qtail
    r = qr(qhead);
    cc = qc(qhead);
    qhead = qhead + int32(1);
    d_here = clear_map(r, cc);

    for k = int32(1):int32(8)
        nr = r + DR(k);
        nc = cc + DC(k);
        if nr < 1 || nr > N_int || nc < 1 || nc > N_int
            continue;
        end
        if occ_map(nr, nc) > 0
            continue;
        end
        new_d = d_here + COSTS(k);
        if new_d < clear_map(nr, nc)
            clear_map(nr, nc) = new_d;
            if qtail <= MAX_Q
                qr(qtail) = nr;
                qc(qtail) = nc;
                qtail = qtail + int32(1);
            end
        end
    end
end
end

%% =====================================================================
%% Local helpers — Hybrid A* primitives, grid lookup, math
%% =====================================================================

function inside = pose_in_goal_box(ex, ey, eyaw, bx, by, byaw, ...
                                    box_l, box_w, box_tol, ego_l, ego_w)
%#codegen
c_e = cos(eyaw); s_e = sin(eyaw);
c_b = cos(byaw); s_b = sin(byaw);

half_w = ego_w * 0.5;
box_half_w = box_w * 0.5;

cx_loc = [0.0,   0.0,   ego_l, ego_l];
cy_loc = [+half_w, -half_w, +half_w, -half_w];

inside = true;
for k = 1:4
    gx_c = ex + c_e * cx_loc(k) - s_e * cy_loc(k);
    gy_c = ey + s_e * cx_loc(k) + c_e * cy_loc(k);
    dx = gx_c - bx;
    dy = gy_c - by;
    lx =  c_b * dx + s_b * dy;
    ly = -s_b * dx + c_b * dy;
    if lx < -box_tol || lx > box_l + box_tol || ...
       ly < -(box_half_w + box_tol) || ly > (box_half_w + box_tol)
        inside = false;
        return;
    end
end
end

function h = lookup_h(x, y, h_grid)
%#codegen
N_grid = 200;
RES = 0.5;
X_MIN = 0.0;
X_MAX = 100.0;
Y_MIN = -100.0;
Y_MAX = 0.0;
if x < X_MIN || x > X_MAX || y < Y_MIN || y > Y_MAX
    h = double(1.0e9);
    return;
end
col = floor((x - X_MIN) / RES) + 1;
row = floor((Y_MAX - y) / RES) + 1;
if row < 1 || row > N_grid || col < 1 || col > N_grid
    h = double(1.0e9);
    return;
end
h = double(h_grid(int32(row), int32(col)));
end

function [nx_o, ny_o, nyaw_o] = bicycle_step(x, y, yaw, steer, ds, L)
%#codegen
if abs(steer) < 1.0e-6
    nx_o = x + ds * cos(yaw);
    ny_o = y + ds * sin(yaw);
    nyaw_o = yaw;
else
    beta = ds * tan(steer) / L;
    R    = L / tan(steer);
    nx_o = x + R * (sin(yaw + beta) - sin(yaw));
    ny_o = y - R * (cos(yaw + beta) - cos(yaw));
    nyaw_o = yaw + beta;
end
nyaw_o = wrap_pi(nyaw_o);
end

function col = is_collision_segment(x1, y1, x2, y2, occ_map)
%#codegen
col = false;
for k = int32(0):int32(8)
    t = double(k) / 8.0;
    x = x1 + t * (x2 - x1);
    y = y1 + t * (y2 - y1);
    if is_occupied_pose(x, y, occ_map)
        col = true;
        return;
    end
end
end

function occ = is_occupied_pose(x, y, occ_map)
%#codegen
N_grid = 200;
RES = 0.5;
X_MIN = 0.0;
X_MAX = 100.0;
Y_MIN = -100.0;
Y_MAX = 0.0;

occ = false;
if x < X_MIN || x > X_MAX || y < Y_MIN || y > Y_MAX
    occ = true;
    return;
end
col = floor((x - X_MIN) / RES) + 1;
row = floor((Y_MAX - y) / RES) + 1;
if row < 1 || row > N_grid || col < 1 || col > N_grid
    occ = true;
    return;
end
if occ_map(row, col) > 0
    occ = true;
end
end

function a = wrap_pi(a)
%#codegen
while a > pi;  a = a - 2.0 * pi; end
while a < -pi; a = a + 2.0 * pi; end
end

function a = wrap_2pi(a)
%#codegen
while a < 0.0;       a = a + 2.0 * pi; end
while a >= 2.0 * pi; a = a - 2.0 * pi; end
end

function d = angle_diff(a, b)
%#codegen
d = wrap_pi(a - b);
end

function s = sign_default(v, fallback)
%#codegen
if v > 0
    s = int8(1);
elseif v < 0
    s = int8(-1);
else
    s = fallback;
end
end

function c = map_const_local()
%#codegen
c.N        = int32(200);
c.RES      = 0.5;
c.X_MIN    = 0.0;
c.X_MAX    = 100.0;
c.Y_MIN    = -100.0;
c.Y_MAX    = 0.0;
c.TRUCK_W  = 2.48;
c.TRUCK_L  = 11.5;
c.EGO_W    = 1.9;
c.EGO_L    = 4.7;
c.WHEELBASE = 2.8;
c.SAFETY_MARGIN = 0.8;
c.CLEAR_MAX = 3.0;
c.W_CLEAR   = 1.2;
c.PARK_BOX_L = 6.0;
c.PARK_BOX_W = 3.0;
c.PARK_TOL   = 0.05;
end
