function [path_x, path_y, path_yaw, path_len] = hybrid_astar_plan( ...
        sx, sy, syaw, gx, gy, gyaw, occ_map)
%HYBRID_ASTAR_PLAN  From-scratch Hybrid A* planner for Day4_5 Scenario 1.
%
%   [path_x, path_y, path_yaw, path_len] = hybrid_astar_plan(sx, sy, syaw,
%                                                            gx, gy, gyaw,
%                                                            occ_map)
%
%   Inputs
%       sx, sy, syaw : ego start pose in world frame (m, rad).
%       gx, gy, gyaw : goal pose (T00 rear bumper or derived) (m, rad).
%       occ_map      : 200x200 uint8 occupancy grid from add_obstacle
%                      (matches map_const.m: 0.5 m/cell, x[0,100], y[-100,0]).
%
%   Outputs
%       path_x, path_y, path_yaw : 1xMAX_PATH double, valid first PATH_LEN
%                                  entries (start -> goal).  Rest is zero.
%       path_len                 : int32, 0 if planning failed.
%
%   Algorithm (plan item 3):
%     - Bicycle-model motion primitives in both directions.
%     - Steer set: 7 angles symmetric around 0.  Combined with +/- step gives
%       14 forward+reverse actions per expansion (reverse is essential for
%       parking maneuvers).
%     - State key: (round(x/POS_RES), round(y/POS_RES), round(yaw/YAW_RES)).
%     - Cost g = sum of step lengths weighted by (1 + 0.2|delta|) and a
%       direction-switch penalty.
%     - Heuristic h = euclidean + W_YAW * |angle_diff(gyaw, yaw)|.
%     - Termination: hypot(g-cur) < GOAL_TOL && |yaw_diff| < YAW_TOL.
%     - Collision check: sample 9 points along each segment and call
%       is_occupied_pose (inflates ego by half-width + safety margin).
%
%#codegen

MAX_NODES = int32(20000);
MAX_PATH  = int32(300);
N_STEER   = int32(7);
N_ACTION  = int32(14);   % 7 steer x 2 direction

% Hyper-parameters (plan item 3 defaults — to be tuned later).
WHEELBASE   = 2.8;
STEP_DIST   = 1.0;             % 2 cells / step at 0.5 m/cell
STEERS = [-0.50, -0.32, -0.16, 0.0, 0.16, 0.32, 0.50];
SWITCH_PENALTY = 4.0;          % direction reversal cost
POS_RES = 0.5;
YAW_RES = pi / 12.0;
GOAL_TOL = 1.5;
YAW_TOL  = 0.45;
W_HEUR   = 1.3;                % weighted A* gain on grid+yaw heuristic
W_YAW    = 1.0;

% Pre-bake two grids:
%   h_grid     : obstacle-aware admissible heuristic (free-space distance
%                from each cell to the goal cell).
%   clear_map  : distance transform — for each cell, the distance to the
%                nearest occupied cell.  Used to shape step_cost so the
%                planner pulls the path toward the corridor centre rather
%                than hugging the inflated obstacle edge.
h_grid    = compute_grid_heuristic(occ_map, gx, gy);
clear_map = compute_clearance(occ_map);

clear_const = map_const();
CLEAR_MAX = clear_const.CLEAR_MAX;
W_CLEAR   = clear_const.W_CLEAR;

path_x   = zeros(1, MAX_PATH);
path_y   = zeros(1, MAX_PATH);
path_yaw = zeros(1, MAX_PATH);
path_len = int32(0);

% Node arrays (struct-of-arrays for codegen).
nx  = zeros(MAX_NODES, 1);
ny  = zeros(MAX_NODES, 1);
nyaw = zeros(MAX_NODES, 1);
ng  = zeros(MAX_NODES, 1);
nf  = zeros(MAX_NODES, 1);
nparent = zeros(MAX_NODES, 1, 'int32');
ndir    = zeros(MAX_NODES, 1, 'int8');   % +1 forward, -1 reverse
nopen   = false(MAX_NODES, 1);
nclosed = false(MAX_NODES, 1);

% Hash keys for duplicate detection (kept aligned with node arrays).
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

for iter = 1:MAX_NODES
    % Pop best open node (linear scan — codegen-friendly).
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

    if hypot(gx - cx, gy - cy) < GOAL_TOL && abs(angle_diff(gyaw, cyaw)) < YAW_TOL
        goal_idx = cur;
        break;
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

        if is_collision_segment(cx, cy, nx_n, ny_n, occ_map)
            continue;
        end

        % Discrete key.
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
        % Clearance shaping: pay extra when close to an obstacle.
        clr = lookup_h(nx_n, ny_n, clear_map);
        if clr < CLEAR_MAX
            step_cost = step_cost + W_CLEAR * (CLEAR_MAX - clr);
        end
        new_g = ng(cur) + step_cost;
        h_grid_n = lookup_h(nx_n, ny_n, h_grid);
        if h_grid_n >= 1.0e9
            % Cell unreachable on grid -> skip (would mislead the search).
            continue;
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

% Trace back goal -> start.
tmp_x   = zeros(MAX_PATH, 1);
tmp_y   = zeros(MAX_PATH, 1);
tmp_yaw = zeros(MAX_PATH, 1);
cnt = int32(0);
idx = goal_idx;
while idx > int32(0) && cnt < MAX_PATH
    cnt = cnt + int32(1);
    tmp_x(cnt)   = nx(idx);
    tmp_y(cnt)   = ny(idx);
    tmp_yaw(cnt) = nyaw(idx);
    idx = nparent(idx);
end

for k = int32(1):cnt
    src = cnt - k + int32(1);
    path_x(k)   = tmp_x(src);
    path_y(k)   = tmp_y(src);
    path_yaw(k) = tmp_yaw(src);
end
path_len = cnt;

end

%% ===== helpers =====

function h = lookup_h(x, y, h_grid)
%#codegen
c = map_const();
N = double(c.N);
res = c.RES;
if x < c.X_MIN || x > c.X_MAX || y < c.Y_MIN || y > c.Y_MAX
    h = double(1.0e9);
    return;
end
col = floor((x - c.X_MIN) / res) + 1;
row = floor((c.Y_MAX - y) / res) + 1;
if row < 1 || row > N || col < 1 || col > N
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
% Simple cell lookup.  Ego footprint inflation is already baked into
% occ_map by add_obstacle (margin = EGO_W/2 + SAFETY_MARGIN), so we
% must NOT inflate again here — that would double-cover the buffer
% and collapse legitimate corridors.
c = map_const();
N = double(c.N);
res = c.RES;

occ = false;
if x < c.X_MIN || x > c.X_MAX || y < c.Y_MIN || y > c.Y_MAX
    occ = true;
    return;
end
col = floor((x - c.X_MIN) / res) + 1;
row = floor((c.Y_MAX - y) / res) + 1;
if row < 1 || row > N || col < 1 || col > N
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
