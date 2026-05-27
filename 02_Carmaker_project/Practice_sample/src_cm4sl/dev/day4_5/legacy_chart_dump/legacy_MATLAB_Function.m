function [desired_ax, steer_fl, steer_fr, path_x_dbg, path_y_dbg, path_len_dbg] = Parking(ego_x, ego_y, ego_yaw, ego_v, start_point, finish_point, goal_yaw, occ_map)
%#codegen

MAX_PATH = 220;

persistent path_x path_y path_len plan_ready tick last_goal_x last_goal_y hold_count prev_steer

if isempty(path_x)
    path_x = zeros(MAX_PATH, 1);
    path_y = zeros(MAX_PATH, 1);
    path_len = 0;

    plan_ready = false;
    tick = 1000;
    last_goal_x = 1.0e9;
    last_goal_y = 1.0e9;
    hold_count = 0;
    prev_steer = 0.0;
end

if abs(ego_yaw) > 2.0 * pi
    ego_yaw = ego_yaw * pi / 180.0;
end

if abs(goal_yaw) > 2.0 * pi
    goal_yaw = goal_yaw * pi / 180.0;
end

goal_x = finish_point(1);
goal_y = finish_point(2);

tick = tick + 1;

need_replan = false;

if ~plan_ready && tick > 20
    need_replan = true;
end

if abs(goal_x - last_goal_x) > 0.2 || abs(goal_y - last_goal_y) > 0.2
    need_replan = true;
end

if need_replan
    was_plan_ready = plan_ready;

    plan_sx = ego_x;
    plan_sy = ego_y;

    if plan_sx < 1.0
        plan_sx = 1.0;
    elseif plan_sx > 99.0
        plan_sx = 99.0;
    end

    if plan_sy > -1.0
        plan_sy = -1.0;
    elseif plan_sy < -99.0
        plan_sy = -99.0;
    end

    [px_raw, py_raw, raw_len] = grid_astar_plan(plan_sx, plan_sy, goal_x, goal_y, occ_map);

    valid_path_found = false;

    if raw_len >= 2 && path_is_collision_free(px_raw, py_raw, raw_len, occ_map)
        [px_smooth, py_smooth, smooth_len] = safe_smooth_path(px_raw, py_raw, raw_len, occ_map);

        if smooth_len >= 2 && path_is_collision_free(px_smooth, py_smooth, smooth_len, occ_map)
            path_x = px_smooth;
            path_y = py_smooth;
            path_len = smooth_len;
            valid_path_found = true;
        else
            path_x = px_raw;
            path_y = py_raw;
            path_len = raw_len;
            valid_path_found = true;
        end
    end

    if valid_path_found && path_is_collision_free(path_x, path_y, path_len, occ_map)
        path_x(1) = ego_x;
        path_y(1) = ego_y;

        if path_is_collision_free(path_x, path_y, path_len, occ_map)
            plan_ready = true;

            if ~was_plan_ready
                hold_count = 5;
            else
                hold_count = 0;
            end
        else
            plan_ready = false;
            path_len = 2;
            path_x(:) = 0.0;
            path_y(:) = 0.0;
            path_x(1) = ego_x;
            path_y(1) = ego_y;
            path_x(2) = ego_x;
            path_y(2) = ego_y;
        end
    else
        if ~plan_ready
            path_x(:) = 0.0;
            path_y(:) = 0.0;
            path_x(1) = ego_x;
            path_y(1) = ego_y;
            path_x(2) = ego_x;
            path_y(2) = ego_y;
            path_len = 2;
        end
    end

    tick = 0;
    last_goal_x = goal_x;
    last_goal_y = goal_y;
end

if ~plan_ready || path_len < 2 || ~path_is_collision_free(path_x, path_y, path_len, occ_map)
    desired_ax = saturate(1.2 * (0.0 - ego_v), -3.0, 0.8);
    steer_fl = 0.0;
    steer_fr = 0.0;

    path_x_dbg = path_x;
    path_y_dbg = path_y;
    path_len_dbg = path_len;

    desired_ax = desired_ax + 0.0 * start_point(1);
    return;
end

if hold_count > 0
    hold_count = hold_count - 1;

    desired_ax = saturate(1.2 * (0.0 - ego_v), -3.0, 0.8);
    steer_fl = 0.0;
    steer_fr = 0.0;

    path_x_dbg = path_x;
    path_y_dbg = path_y;
    path_len_dbg = path_len;

    desired_ax = desired_ax + 0.0 * start_point(1);
    return;
end

[desired_ax, steer_cmd] = pure_pursuit_control(ego_x, ego_y, ego_yaw, ego_v, ...
    goal_x, goal_y, goal_yaw, path_x, path_y, path_len);

max_steer_rate = 0.03;
steer_cmd = saturate(steer_cmd, prev_steer - max_steer_rate, prev_steer + max_steer_rate);
prev_steer = steer_cmd;

steer_fl = steer_cmd;
steer_fr = steer_cmd;

path_x_dbg = path_x;
path_y_dbg = path_y;
path_len_dbg = path_len;

desired_ax = desired_ax + 0.0 * start_point(1);

end

function [path_x, path_y, path_len] = grid_astar_plan(sx, sy, gx, gy, occ_map)

N = 100;
MAX_PATH = 220;
MAX_NODES = 10000;

path_x = zeros(MAX_PATH, 1);
path_y = zeros(MAX_PATH, 1);
path_len = 0;

[start_r, start_c, valid_s] = world_to_grid(sx, sy);
[goal_r, goal_c, valid_g] = world_to_grid(gx, gy);

if ~valid_s || ~valid_g
    return;
end

if occ_map(start_r, start_c) > 0.5 || occ_map(goal_r, goal_c) > 0.5
    return;
end

open = false(N, N);
closed = false(N, N);

g_cost = 1.0e9 * ones(N, N);
f_cost = 1.0e9 * ones(N, N);

parent_r = zeros(N, N);
parent_c = zeros(N, N);

g_cost(start_r, start_c) = 0.0;
f_cost(start_r, start_c) = heuristic_grid(start_r, start_c, goal_r, goal_c);
open(start_r, start_c) = true;

found = false;

dr = [-1; -1; -1;  0; 0; 1; 1; 1];
dc = [-1;  0;  1; -1; 1;-1; 0; 1];

for iter = 1:MAX_NODES
    cur_r = 0;
    cur_c = 0;
    best_f = 1.0e9;

    for r = 1:N
        for c = 1:N
            if open(r, c) && f_cost(r, c) < best_f
                best_f = f_cost(r, c);
                cur_r = r;
                cur_c = c;
            end
        end
    end

    if cur_r == 0
        break;
    end

    if cur_r == goal_r && cur_c == goal_c
        found = true;
        break;
    end

    open(cur_r, cur_c) = false;
    closed(cur_r, cur_c) = true;

    for k = 1:8
        nr = cur_r + dr(k);
        nc = cur_c + dc(k);

        if nr < 1 || nr > N || nc < 1 || nc > N
            continue;
        end

        if closed(nr, nc)
            continue;
        end

        if occ_map(nr, nc) > 0.5
            continue;
        end

        if k == 1 || k == 3 || k == 6 || k == 8
            step_cost = 1.4142;
        else
            step_cost = 1.0;
        end

        new_g = g_cost(cur_r, cur_c) + step_cost;

        if ~open(nr, nc) || new_g < g_cost(nr, nc)
            parent_r(nr, nc) = cur_r;
            parent_c(nr, nc) = cur_c;
            g_cost(nr, nc) = new_g;
            f_cost(nr, nc) = new_g + heuristic_grid(nr, nc, goal_r, goal_c);
            open(nr, nc) = true;
        end
    end
end

if ~found
    return;
end

tmp_x = zeros(MAX_PATH, 1);
tmp_y = zeros(MAX_PATH, 1);

cnt = 0;
r = goal_r;
c = goal_c;

while cnt < MAX_PATH
    cnt = cnt + 1;

    [wx, wy] = grid_to_world(r, c);
    tmp_x(cnt) = wx;
    tmp_y(cnt) = wy;

    if r == start_r && c == start_c
        break;
    end

    pr = parent_r(r, c);
    pc = parent_c(r, c);

    if pr == 0 || pc == 0
        break;
    end

    r = pr;
    c = pc;
end

for i = 1:cnt
    src = cnt - i + 1;
    path_x(i) = tmp_x(src);
    path_y(i) = tmp_y(src);
end

path_len = cnt;

end

function h = heuristic_grid(r, c, gr, gc)

dr = double(gr - r);
dc = double(gc - c);
h = sqrt(dr * dr + dc * dc);

end

function [path_x2, path_y2, path_len2] = safe_smooth_path(path_x, path_y, path_len, occ_map)

MAX_PATH = 220;

path_x2 = zeros(MAX_PATH, 1);
path_y2 = zeros(MAX_PATH, 1);
path_len2 = 0;

if path_len < 2
    return;
end

path_len2 = path_len;

for i = 1:MAX_PATH
    path_x2(i) = path_x(i);
    path_y2(i) = path_y(i);
end

alpha = 0.15;

for iter = 1:8
    for i = 2:(MAX_PATH - 1)
        if i < path_len2
            old_x = path_x2(i);
            old_y = path_y2(i);

            new_x = old_x + alpha * 0.5 * ...
                (path_x2(i - 1) + path_x2(i + 1) - 2.0 * old_x);
            new_y = old_y + alpha * 0.5 * ...
                (path_y2(i - 1) + path_y2(i + 1) - 2.0 * old_y);

            if is_free_segment(path_x2(i - 1), path_y2(i - 1), new_x, new_y, occ_map) && ...
                    is_free_segment(new_x, new_y, path_x2(i + 1), path_y2(i + 1), occ_map)
                path_x2(i) = new_x;
                path_y2(i) = new_y;
            end
        end
    end
end

end

function free = path_is_collision_free(path_x, path_y, path_len, occ_map)

free = true;

if path_len < 2
    free = false;
    return;
end

for i = 1:219
    if i < path_len
        if ~is_free_segment(path_x(i), path_y(i), path_x(i + 1), path_y(i + 1), occ_map)
            free = false;
            return;
        end
    end
end

end

function free = is_free_segment(x1, y1, x2, y2, occ_map)

free = true;

dist = hypot(x2 - x1, y2 - y1);
n = ceil(dist / 0.25);

if n < 1
    n = 1;
end

for k = 0:80
    if k <= n
        t = double(k) / double(n);
        x = x1 + t * (x2 - x1);
        y = y1 + t * (y2 - y1);

        if ~is_free_point(x, y, occ_map)
            free = false;
            return;
        end
    end
end

end

function free = is_free_point(x, y, occ_map)

free = false;

[row, col, valid] = world_to_grid(x, y);

if ~valid
    return;
end

if occ_map(row, col) <= 0.5
    free = true;
end

end

function [desired_ax, steer_cmd] = pure_pursuit_control(ego_x, ego_y, ego_yaw, ego_v, goal_x, goal_y, goal_yaw, path_x, path_y, path_len)

wheelbase = 2.8;
max_steer = 0.45;

dist_goal = hypot(goal_x - ego_x, goal_y - ego_y);
yaw_err_goal = angle_diff_local(goal_yaw, ego_yaw);

if path_len < 2
    desired_ax = saturate(1.2 * (0.0 - ego_v), -3.0, 0.8);
    steer_cmd = 0.0;
    return;
end

if dist_goal < 2.0
    desired_ax = saturate(1.2 * (0.0 - ego_v), -3.0, 0.8);
    steer_cmd = saturate(0.8 * yaw_err_goal, -max_steer, max_steer);
    return;
end

nearest = 1;
best_d = 1.0e12;

for i = 1:220
    if i <= path_len
        d = (path_x(i) - ego_x)^2 + (path_y(i) - ego_y)^2;
        if d < best_d
            best_d = d;
            nearest = i;
        end
    end
end

lookahead = 2.0 + 0.25 * abs(ego_v);

if lookahead > 5.0
    lookahead = 5.0;
end

target = nearest;
acc_dist = 0.0;

for i = nearest:219
    if i >= path_len
        break;
    end

    seg = hypot(path_x(i + 1) - path_x(i), path_y(i + 1) - path_y(i));
    acc_dist = acc_dist + seg;
    target = i + 1;

    if acc_dist >= lookahead
        break;
    end
end

tx = path_x(target);
ty = path_y(target);

dx = tx - ego_x;
dy = ty - ego_y;

c = cos(ego_yaw);
s = sin(ego_yaw);

local_x = c * dx + s * dy;
local_y = -s * dx + c * dy;

if local_x < 0.2
    local_x = 0.2;
end

steer_cmd = atan2(2.0 * wheelbase * local_y, lookahead^2);
steer_cmd = saturate(steer_cmd, -max_steer, max_steer);

if dist_goal > 15.0
    desired_v = 2.5;
elseif dist_goal > 6.0
    desired_v = 1.5;
else
    desired_v = 0.7;
end

desired_ax = 0.8 * (desired_v - ego_v);
desired_ax = saturate(desired_ax, -3.0, 1.5);

end

function [row, col, valid] = world_to_grid(x, y)

valid = true;

if x < 0.0 || x > 100.0 || y < -100.0 || y > 0.0
    row = 1;
    col = 1;
    valid = false;
    return;
end

col = floor(x) + 1;
row = floor(-y) + 1;

if col < 1
    col = 1;
elseif col > 100
    col = 100;
end

if row < 1
    row = 1;
elseif row > 100
    row = 100;
end

end

function [x, y] = grid_to_world(row, col)

x = double(col) - 0.5;
y = -(double(row) - 0.5);

end

function y = saturate(x, lo, hi)

y = x;

if y < lo
    y = lo;
elseif y > hi
    y = hi;
end

end

function a = wrap_pi_local(a)

while a > pi
    a = a - 2.0 * pi;
end

while a < -pi
    a = a + 2.0 * pi;
end

end

function d = angle_diff_local(a, b)

d = wrap_pi_local(a - b);

end