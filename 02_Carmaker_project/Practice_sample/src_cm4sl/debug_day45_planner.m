function debug_day45_planner()
% Standalone reproduction of the Day4_5 Scenario 1 Hybrid A* planner
% Replicates chart_18 + chart_45 logic to diagnose why the path is not
% generated correctly.

%% --- Scenario inputs (from system_root constants + TestRun) ---
ego_x = 0;
ego_y = -20;
ego_yaw = 0;        % rad
ego_v = 0;

start_point = [0; -20];
finish_point = [80; -3];
goal_yaw = 0;

% Traffic_size in chart_45: [width length]; root sets [2.48, 11.5]
traffic_size = [2.48, 11.5];

% Traffic objects as defined in TestRun day4_5_scenario1
% [x, y, yaw_deg] — the Subsystem reads ONLY T01..T07; T00 is missing.
traffic_all_deg = [
    76.1, -5,  90;   % T00 (NOT included in current build)
    40,  -12,  90;   % T01
    40,  -24,  90;   % T02
    40,  -36,  90;   % T03
    40,  -48,  90;   % T04
    39,  -50,   0;   % T05
    51,  -50,   0;   % T06
    63,  -50,   0;   % T07
];

% Build a "broken" traffic_info matching what chart_45 currently sees: T01..T07
broken = traffic_all_deg(2:8, :);
traffic_info_broken = zeros(21, 1);
for k = 1:7
    traffic_info_broken((k-1)*3 + 1) = broken(k,1);
    traffic_info_broken((k-1)*3 + 2) = broken(k,2);
    traffic_info_broken((k-1)*3 + 3) = deg2rad(broken(k,3));  % CarMaker yields rad
end

% Build a "fixed" traffic_info that includes all 8 trucks (T00..T07)
fixed = traffic_all_deg;
traffic_info_fixed = zeros(24, 1);
for k = 1:8
    traffic_info_fixed((k-1)*3 + 1) = fixed(k,1);
    traffic_info_fixed((k-1)*3 + 2) = fixed(k,2);
    traffic_info_fixed((k-1)*3 + 3) = deg2rad(fixed(k,3));
end

%% --- Build the base map (frame border from chart_36) ---
base_map = zeros(200, 200);
base_map(1, :)   = 1;
base_map(end, :) = 1;
base_map(:, 1)   = 1;
base_map(:, end) = 1;

%% --- Add obstacles two ways: broken (i=1:7) and fixed (i=1:8) ---
occ_broken = add_obstacle_(base_map, traffic_info_broken, traffic_size, 7);
occ_fixed  = add_obstacle_(base_map, traffic_info_fixed, traffic_size, 8);

fprintf('Occupied cells (broken, 7 trucks): %d\n', sum(occ_broken(:) > 0.5));
fprintf('Occupied cells (fixed,  8 trucks): %d\n', sum(occ_fixed(:)  > 0.5));

%% --- Run the planner with both maps and report ---
fprintf('\n--- BROKEN map (current state, T00 missing) ---\n');
tic;
[px_b, py_b, pyaw_b, plen_b] = hybrid_astar_plan(ego_x, ego_y, ego_yaw, ...
    finish_point(1), finish_point(2), goal_yaw, occ_broken);
elapsed_b = toc;
fprintf('plan time: %.2fs, path_len = %d\n', elapsed_b, plen_b);
if plen_b >= 2
    fprintf('first 3 pts: (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f)\n', ...
        px_b(1), py_b(1), px_b(2), py_b(2), px_b(3), py_b(3));
    fprintf('last 3 pts:  (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f)\n', ...
        px_b(plen_b-2), py_b(plen_b-2), px_b(plen_b-1), py_b(plen_b-1), px_b(plen_b), py_b(plen_b));
end

fprintf('\n--- FIXED map (T00 included) ---\n');
tic;
[px_f, py_f, pyaw_f, plen_f] = hybrid_astar_plan(ego_x, ego_y, ego_yaw, ...
    finish_point(1), finish_point(2), goal_yaw, occ_fixed);
elapsed_f = toc;
fprintf('plan time: %.2fs, path_len = %d\n', elapsed_f, plen_f);
if plen_f >= 2
    fprintf('first 3 pts: (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f)\n', ...
        px_f(1), py_f(1), px_f(2), py_f(2), px_f(3), py_f(3));
    fprintf('last 3 pts:  (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f)\n', ...
        px_f(plen_f-2), py_f(plen_f-2), px_f(plen_f-1), py_f(plen_f-1), px_f(plen_f), py_f(plen_f));
end

%% --- Plot ---
figure('Name','Day4_5 path debug');
hold on; axis equal; grid on;
imagesc([-100 100], [100 -100], occ_fixed); colormap(flipud(gray)); axis xy;
xlim([-5 105]); ylim([-105 5]);
plot(ego_x, ego_y, 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(finish_point(1), finish_point(2), 'r*', 'MarkerSize', 14);
if plen_b >= 2
    plot(px_b(1:plen_b), py_b(1:plen_b), 'b-', 'LineWidth', 1.5, 'DisplayName','broken');
end
if plen_f >= 2
    plot(px_f(1:plen_f), py_f(1:plen_f), 'm-', 'LineWidth', 1.5, 'DisplayName','fixed');
end
legend('Location','best');
title('Hybrid A* path: broken (blue) vs fixed (magenta)');
saveas(gcf, fullfile(fileparts(mfilename('fullpath')), 'debug_day45_paths.png'));
fprintf('\nSaved plot to debug_day45_paths.png\n');

end

%% ===== chart_45 add_obstacle_ but parameterized over number of trucks =====
function y = add_obstacle_(map, traffic_info, traffic_size, n_trucks)
y = map;
res = 1.0;
x_min = -100.0;
y_max =  100.0;
n = 200;
veh_w = traffic_size(1);
veh_l = traffic_size(2);
half_w = veh_w * 0.5 + 1.0;
half_l = veh_l * 0.5 + 1.0;
for i = 1:n_trucks
    idx = (i - 1) * 3;
    px = traffic_info(idx + 1);
    py = traffic_info(idx + 2);
    yaw = traffic_info(idx + 3);
    if abs(px) < 1e-6 && abs(py) < 1e-6 && abs(yaw) < 1e-6
        continue;
    end
    c_yaw = cos(yaw); s_yaw = sin(yaw);
    for row = 1:n
        wy = y_max - (double(row) - 0.5) * res;
        for col = 1:n
            wx = x_min + (double(col) - 0.5) * res;
            dx = wx - px;
            dy = wy - py;
            local_x =  dx * c_yaw + dy * s_yaw;
            local_y = -dx * s_yaw + dy * c_yaw;
            if abs(local_x) <= half_l && abs(local_y) <= half_w
                y(row, col) = 1;
            end
        end
    end
end
end

%% ===== chart_18 hybrid_astar_plan + helpers (verbatim) =====
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
    cur = 0; best_f = 1e12;
    for i = 1:MAX_NODES
        if node_open(i) && node_f(i) < best_f
            best_f = node_f(i); cur = i;
        end
    end
    if cur == 0, break; end
    node_open(cur) = false; node_closed(cur) = true;
    cx = node_x(cur); cy = node_y(cur); cyaw = node_yaw(cur);
    dist_goal = hypot(gx - cx, gy - cy);
    yaw_err = abs(angle_diff_local(gyaw, cyaw));
    if dist_goal < goal_tol && yaw_err < yaw_tol
        goal_idx = cur; break;
    end
    for a = 1:N_ACTION
        delta = steers(a);
        [nx, ny, nyaw] = bicycle_step(cx, cy, cyaw, delta, step_dist, wheelbase);
        if is_collision_segment(cx, cy, nx, ny, occ_map), continue; end
        nkey_x = round(nx / pos_res);
        nkey_y = round(ny / pos_res);
        nkey_yaw = round(wrap_2pi_local(nyaw) / yaw_res);
        duplicated = false; dup_idx = 0;
        for j = 1:node_count
            jkey_x = round(node_x(j) / pos_res);
            jkey_y = round(node_y(j) / pos_res);
            jkey_yaw = round(wrap_2pi_local(node_yaw(j)) / yaw_res);
            if jkey_x == nkey_x && jkey_y == nkey_y && jkey_yaw == nkey_yaw
                duplicated = true; dup_idx = j; break;
            end
        end
        new_g = node_g(cur) + step_dist * (1.0 + 0.2 * abs(delta));
        h = hypot(gx - nx, gy - ny) + 1.5 * abs(angle_diff_local(gyaw, nyaw));
        new_f = new_g + 1.4 * h;
        if duplicated
            if node_closed(dup_idx), continue; end
            if new_g >= node_g(dup_idx), continue; end
            node_x(dup_idx) = nx; node_y(dup_idx) = ny; node_yaw(dup_idx) = nyaw;
            node_g(dup_idx) = new_g; node_f(dup_idx) = new_f;
            node_parent(dup_idx) = cur; node_open(dup_idx) = true;
        else
            if node_count >= MAX_NODES, break; end
            node_count = node_count + 1;
            node_x(node_count) = nx; node_y(node_count) = ny; node_yaw(node_count) = nyaw;
            node_g(node_count) = new_g; node_f(node_count) = new_f;
            node_parent(node_count) = cur; node_open(node_count) = true;
        end
    end
end
if goal_idx == 0, return; end
tmp_x = zeros(MAX_PATH, 1);
tmp_y = zeros(MAX_PATH, 1);
tmp_yaw = zeros(MAX_PATH, 1);
cnt = 0; idx = goal_idx;
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

function [nx, ny, nyaw] = bicycle_step(x, y, yaw, steer, ds, wheelbase)
if abs(steer) < 1e-6
    nx = x + ds * cos(yaw); ny = y + ds * sin(yaw); nyaw = yaw;
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
    x = x1 + t * (x2 - x1); y = y1 + t * (y2 - y1);
    if is_occupied_pose(x, y, occ_map), col = true; return; end
end
end

function occ = is_occupied_pose(x, y, occ_map)
occ = false;
map_x_min = -100.0; map_x_max = 100.0;
map_y_min = -100.0; map_y_max = 100.0;
road_x_min = 0.0; road_x_max = 100.0;
road_y_min = -100.0; road_y_max = -2.0;
if x < road_x_min || x > road_x_max || y < road_y_min || y > road_y_max
    occ = true; return;
end
nr = 200; nc = 200;
res_x = (map_x_max - map_x_min) / double(nc);
res_y = (map_y_max - map_y_min) / double(nr);
res = max(res_x, res_y);
col = floor((x - map_x_min) / res_x) + 1;
row = floor((map_y_max - y) / res_y) + 1;
if row < 1 || row > nr || col < 1 || col > nc
    occ = true; return;
end
inflate = ceil(0.8 / res);
r1 = max(1, row - inflate); r2 = min(nr, row + inflate);
c1 = max(1, col - inflate); c2 = min(nc, col + inflate);
for r = r1:r2
    for c = c1:c2
        if occ_map(r, c) > 0.5, occ = true; return; end
    end
end
end

function a = wrap_pi_local(a)
while a > pi,  a = a - 2.0 * pi; end
while a < -pi, a = a + 2.0 * pi; end
end

function a = wrap_2pi_local(a)
while a < 0.0,        a = a + 2.0 * pi; end
while a >= 2.0 * pi,  a = a - 2.0 * pi; end
end

function d = angle_diff_local(a, b)
d = wrap_pi_local(a - b);
end
