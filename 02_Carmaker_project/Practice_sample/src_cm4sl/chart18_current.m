function [desired_ax, steer_fl, steer_fr, obs_count, path_len_dbg, viz_map] = fcn(ego_x, ego_y, ego_yaw, ego_v, start_point, finish_point, goal_yaw, occ_map)
%#codegen

MAX_PATH = 220;

persistent path_x path_y path_yaw path_len tick last_goal_x last_goal_y

if isempty(path_x)
    path_x = zeros(MAX_PATH, 1);
    path_y = zeros(MAX_PATH, 1);
    path_yaw = zeros(MAX_PATH, 1);
    path_len = 0;
    tick = 1000;
    last_goal_x = 1e9;
    last_goal_y = 1e9;
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

obs_count = 0;
for r = 1:200
    for c = 1:200
        if occ_map(r, c) > 0.5
            obs_count = obs_count + 1;
        end
    end
end

need_replan = false;

if path_len < 2
    need_replan = true;
end

if tick > 200
    need_replan = true;
end

if abs(goal_x - last_goal_x) > 0.2 || abs(goal_y - last_goal_y) > 0.2
    need_replan = true;
end

if need_replan
    [px, py, pyaw, plen] = hybrid_astar_plan(ego_x, ego_y, ego_yaw, goal_x, goal_y, goal_yaw, occ_map);

    if plen >= 2
        path_x = px;
        path_y = py;
        path_yaw = pyaw;
        path_len = plen;
    elseif path_len < 2
        % First-time replan failure: seed a single-point "stay-put" path so the
        % controller can decelerate instead of doing pure-pursuit on a stale 2-point line.
        path_x(:) = 0.0;
        path_y(:) = 0.0;
        path_yaw(:) = 0.0;
        path_x(1) = ego_x;
        path_y(1) = ego_y;
        path_yaw(1) = ego_yaw;
        path_x(2) = ego_x;
        path_y(2) = ego_y;
        path_yaw(2) = ego_yaw;
        path_len = 2;
    end
    % If replan failed but a previous valid path exists, keep it.

    tick = 0;
    last_goal_x = goal_x;
    last_goal_y = goal_y;
end

[desired_ax, steer_cmd] = follow_path_controller(ego_x, ego_y, ego_yaw, ego_v, goal_x, goal_y, goal_yaw, path_x, path_y, path_len);

steer_fl = steer_cmd;
steer_fr = steer_cmd;

path_len_dbg = path_len;

viz_map = make_viz_map(occ_map, path_x, path_y, path_len, ego_x, ego_y, goal_x, goal_y);

desired_ax = desired_ax + 0.0 * start_point(1);

end

function [path_x, path_y, path_yaw, path_len] = hybrid_astar_plan(sx, sy, syaw, gx, gy, gyaw, occ_map)

MAX_NODES = 4500;
MAX_PATH = 220;
N_ACTION = 7;

path_x = zeros(MAX_PATH, 1);
path_y = zeros(MAX_PATH, 1);
path_yaw = zeros(MAX_PATH, 1);
path_len = 0;

node_x = zeros(MAX_NODES, 1);
node_y = zeros(MAX_NODES, 1);
node_yaw = zeros(MAX_NODES, 1);
node_g = zeros(MAX_NODES, 1);
node_f = zeros(MAX_NODES, 1);
node_parent = zeros(MAX_NODES, 1);
node_open = false(MAX_NODES, 1);
node_closed = false(MAX_NODES, 1);

steers = [-0.50, -0.32, -0.16, 0.0, 0.16, 0.32, 0.50];

wheelbase = 2.8;
step_dist = 1.2;
pos_res = 1.0;
yaw_res = pi / 8.0;

goal_tol = 2.0;
yaw_tol = 0.8;

node_count = 1;

node_x(1) = sx;
node_y(1) = sy;
node_yaw(1) = wrap_pi_local(syaw);
node_g(1) = 0.0;
node_f(1) = hypot(gx - sx, gy - sy);
node_parent(1) = 0;
node_open(1) = true;

goal_idx = 0;

for iter = 1:MAX_NODES
    cur = 0;
    best_f = 1e12;

    for i = 1:MAX_NODES
        if node_open(i) && node_f(i) < best_f
            best_f = node_f(i);
            cur = i;
        end
    end

    if cur == 0
        break;
    end

    node_open(cur) = false;
    node_closed(cur) = true;

    cx = node_x(cur);
    cy = node_y(cur);
    cyaw = node_yaw(cur);

    dist_goal = hypot(gx - cx, gy - cy);
    yaw_err = abs(angle_diff_local(gyaw, cyaw));

    if dist_goal < goal_tol && yaw_err < yaw_tol
        goal_idx = cur;
        break;
    end

    for a = 1:N_ACTION
        delta = steers(a);

        [nx, ny, nyaw] = bicycle_step(cx, cy, cyaw, delta, step_dist, wheelbase);

        if is_collision_segment(cx, cy, nx, ny, occ_map)
            continue;
        end

        nkey_x = round(nx / pos_res);
        nkey_y = round(ny / pos_res);
        nkey_yaw = round(wrap_2pi_local(nyaw) / yaw_res);

        duplicated = false;
        dup_idx = 0;

        for j = 1:node_count
            jkey_x = round(node_x(j) / pos_res);
            jkey_y = round(node_y(j) / pos_res);
            jkey_yaw = round(wrap_2pi_local(node_yaw(j)) / yaw_res);

            if jkey_x == nkey_x && jkey_y == nkey_y && jkey_yaw == nkey_yaw
                duplicated = true;
                dup_idx = j;
                break;
            end
        end

        new_g = node_g(cur) + step_dist * (1.0 + 0.2 * abs(delta));
        h = hypot(gx - nx, gy - ny) + 1.5 * abs(angle_diff_local(gyaw, nyaw));
        new_f = new_g + 1.4 * h;

        if duplicated
            if node_closed(dup_idx)
                continue;
            end

            if new_g >= node_g(dup_idx)
                continue;
            end

            node_x(dup_idx) = nx;
            node_y(dup_idx) = ny;
            node_yaw(dup_idx) = nyaw;
            node_g(dup_idx) = new_g;
            node_f(dup_idx) = new_f;
            node_parent(dup_idx) = cur;
            node_open(dup_idx) = true;
        else
            if node_count >= MAX_NODES
                break;
            end

            node_count = node_count + 1;
            node_x(node_count) = nx;
            node_y(node_count) = ny;
            node_yaw(node_count) = nyaw;
            node_g(node_count) = new_g;
            node_f(node_count) = new_f;
            node_parent(node_count) = cur;
            node_open(node_count) = true;
        end
    end
end

if goal_idx == 0
    return;
end

tmp_x = zeros(MAX_PATH, 1);
tmp_y = zeros(MAX_PATH, 1);
tmp_yaw = zeros(MAX_PATH, 1);

cnt = 0;
idx = goal_idx;

while idx > 0 && cnt < MAX_PATH
    cnt = cnt + 1;
    tmp_x(cnt) = node_x(idx);
    tmp_y(cnt) = node_y(idx);
    tmp_yaw(cnt) = node_yaw(idx);
    idx = node_parent(idx);
end

for k = 1:cnt
    src = cnt - k + 1;
    path_x(k) = tmp_x(src);
    path_y(k) = tmp_y(src);
    path_yaw(k) = tmp_yaw(src);
end

path_len = cnt;

end

function [desired_ax, steer_cmd] = follow_path_controller(ego_x, ego_y, ego_yaw, ego_v, goal_x, goal_y, goal_yaw, path_x, path_y, path_len)

wheelbase = 2.8;
max_steer = 0.45;

dist_goal = hypot(goal_x - ego_x, goal_y - ego_y);
yaw_err_goal = angle_diff_local(goal_yaw, ego_yaw);

if dist_goal < 2.0
    desired_v = 0.0;
    steer_cmd = saturate(0.8 * yaw_err_goal, -max_steer, max_steer);
    desired_ax = saturate(1.2 * (desired_v - ego_v), -3.0, 0.8);
    return;
end

nearest = 1;
best_d = 1e12;

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
if lookahead > 4.5
    lookahead = 4.5;
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
    desired_v = 1.6;
else
    desired_v = 0.8;
end

desired_ax = 0.8 * (desired_v - ego_v);
desired_ax = saturate(desired_ax, -3.0, 1.5);

end

function viz_map = make_viz_map(occ_map, path_x, path_y, path_len, ego_x, ego_y, goal_x, goal_y)

viz_map = zeros(200, 200);

for r = 1:200
    for c = 1:200
        if occ_map(r, c) > 0.5
            viz_map(r, c) = 1;
        end
    end
end

for i = 1:220
    if i <= path_len
        [row, col, valid] = world_to_grid(path_x(i), path_y(i));

        if valid
            viz_map(row, col) = 2;
        end
    end
end

[row, col, valid] = world_to_grid(goal_x, goal_y);
if valid
    viz_map(row, col) = 4;
end

[row, col, valid] = world_to_grid(ego_x, ego_y);
if valid
    viz_map(row, col) = 3;
end

end

function [nx, ny, nyaw] = bicycle_step(x, y, yaw, steer, ds, wheelbase)

if abs(steer) < 1e-6
    nx = x + ds * cos(yaw);
    ny = y + ds * sin(yaw);
    nyaw = yaw;
else
    beta = ds * tan(steer) / wheelbase;
    radius = wheelbase / tan(steer);

    nx = x + radius * (sin(yaw + beta) - sin(yaw));
    ny = y - radius * (cos(yaw + beta) - cos(yaw));
    nyaw = yaw + beta;
end

nyaw = wrap_pi_local(nyaw);

end

function col = is_collision_segment(x1, y1, x2, y2, occ_map)

col = false;

for k = 0:8
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

occ = false;

map_x_min = -100.0;
map_x_max = 100.0;
map_y_min = -100.0;
map_y_max = 100.0;

road_x_min = 0.0;
road_x_max = 100.0;
road_y_min = -100.0;
road_y_max = 0.0;

if x < road_x_min || x > road_x_max || y < road_y_min || y > road_y_max
    occ = true;
    return;
end

[row, col, valid] = world_to_grid(x, y);

if ~valid
    occ = true;
    return;
end

res = 1.0;
inflate = ceil(0.8 / res);

r1 = max(1, row - inflate);
r2 = min(200, row + inflate);
c1 = max(1, col - inflate);
c2 = min(200, col + inflate);

for r = r1:r2
    for c = c1:c2
        if occ_map(r, c) > 0.5
            occ = true;
            return;
        end
    end
end

end

function [row, col, valid] = world_to_grid(x, y)

map_x_min = -100.0;
map_x_max = 100.0;
map_y_min = -100.0;
map_y_max = 100.0;

valid = true;

if x < map_x_min || x > map_x_max || y < map_y_min || y > map_y_max
    row = 1;
    col = 1;
    valid = false;
    return;
end

res_x = (map_x_max - map_x_min) / 200.0;
res_y = (map_y_max - map_y_min) / 200.0;

col = floor((x - map_x_min) / res_x) + 1;
row = floor((map_y_max - y) / res_y) + 1;

if row < 1 || row > 200 || col < 1 || col > 200
    valid = false;
end

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

function a = wrap_2pi_local(a)

while a < 0.0
    a = a + 2.0 * pi;
end

while a >= 2.0 * pi
    a = a - 2.0 * pi;
end

end

function d = angle_diff_local(a, b)

d = wrap_pi_local(a - b);

end