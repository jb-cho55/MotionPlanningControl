function debug_day45_boundary2()
% Verify the effect of road_y_max change.
% Uses the same planner structure as debug_day45_planner.m, just exposes
% the boundary as an argument so we can compare -2 vs 0.

ego_x = 0; ego_y = -20; ego_yaw = 0;
finish_point = [80; -3];
goal_yaw = 0;
traffic_size = [2.48, 11.5];

traffic_all_deg = [
    76.1, -5,  90;  40, -12, 90;  40, -24, 90;  40, -36, 90;
    40,  -48, 90;  39, -50,  0;  51, -50,  0;  63, -50, 0;
];
broken = traffic_all_deg(2:8, :);
ti = zeros(21, 1);
for k = 1:7
    ti((k-1)*3 + 1) = broken(k,1);
    ti((k-1)*3 + 2) = broken(k,2);
    ti((k-1)*3 + 3) = deg2rad(broken(k,3));
end
base_map = zeros(200, 200);
base_map(1, :)=1; base_map(end, :)=1; base_map(:, 1)=1; base_map(:, end)=1;
occ = add_obstacle_(base_map, ti, traffic_size, 7);

for ymax = [-2 0]
    set_road_y_max(ymax);
    tic;
    [px, py, ~, plen] = hybrid_astar_plan(ego_x, ego_y, ego_yaw, finish_point(1), finish_point(2), goal_yaw, occ);
    t = toc;
    if plen > 0
        ymin_path = min(py(1:plen)); ymax_path = max(py(1:plen));
        fprintf('road_y_max=%+d: plen=%3d (%.2fs), ends (%.1f,%.1f), path y in [%.2f, %.2f]\n', ymax, plen, t, px(plen), py(plen), ymin_path, ymax_path);
    else
        fprintf('road_y_max=%+d: NO PATH (%.2fs)\n', ymax, t);
    end
    if ymax == -2, px_a=px; py_a=py; plen_a=plen; else, px_b=px; py_b=py; plen_b=plen; end
end

figure('Position',[100 100 1100 500]);
hold on; grid on; axis equal;
imagesc([-100 100], [100 -100], occ); colormap(flipud(gray)); axis xy;
xlim([-5 105]); ylim([-25 5]);
if plen_a > 0
    plot(px_a(1:plen_a), py_a(1:plen_a), 'b.-', 'LineWidth',1.5,'DisplayName','road\_y\_max=-2 (current)');
end
if plen_b > 0
    plot(px_b(1:plen_b), py_b(1:plen_b), 'm.-', 'LineWidth',1.5,'DisplayName','road\_y\_max=0 (fixed)');
end
plot(0,-20,'go','MarkerSize',10,'LineWidth',2);
plot(80,-3,'r*','MarkerSize',14);
plot([0 100],[-2 -2],'b--','LineWidth',1);
plot([0 100],[0 0],'m--','LineWidth',1);
legend('Location','south');
title('road_{y,max}: -2 forces path to hug boundary; 0 gives clearance');
saveas(gcf, fullfile(fileparts(mfilename('fullpath')), 'debug_day45_boundary.png'));
end

function set_road_y_max(v)
global ROAD_Y_MAX
ROAD_Y_MAX = v;
end

%% ===== Planner (same as chart_18) with global road_y_max =====
function y = add_obstacle_(map, traffic_info, traffic_size, n_trucks)
y = map;
res = 1.0; x_min = -100.0; y_max = 100.0; n = 200;
veh_w = traffic_size(1); veh_l = traffic_size(2);
half_w = veh_w * 0.5 + 1.0; half_l = veh_l * 0.5 + 1.0;
for i = 1:n_trucks
    idx = (i - 1) * 3;
    px = traffic_info(idx + 1); py = traffic_info(idx + 2); yaw = traffic_info(idx + 3);
    if abs(px) < 1e-6 && abs(py) < 1e-6 && abs(yaw) < 1e-6, continue; end
    c_yaw = cos(yaw); s_yaw = sin(yaw);
    for row = 1:n, wy = y_max - (double(row) - 0.5) * res;
        for col = 1:n, wx = x_min + (double(col) - 0.5) * res;
            dx = wx - px; dy = wy - py;
            local_x = dx*c_yaw + dy*s_yaw; local_y = -dx*s_yaw + dy*c_yaw;
            if abs(local_x) <= half_l && abs(local_y) <= half_w, y(row,col)=1; end
        end
    end
end
end

function [path_x, path_y, path_yaw, path_len] = hybrid_astar_plan(sx, sy, syaw, gx, gy, gyaw, occ_map)
MAX_NODES = 4500; MAX_PATH = 220; N_ACTION = 7;
path_x = zeros(MAX_PATH,1); path_y = zeros(MAX_PATH,1); path_yaw = zeros(MAX_PATH,1); path_len = 0;
node_x = zeros(MAX_NODES,1); node_y = zeros(MAX_NODES,1); node_yaw = zeros(MAX_NODES,1);
node_g = zeros(MAX_NODES,1); node_f = zeros(MAX_NODES,1); node_parent = zeros(MAX_NODES,1);
node_open = false(MAX_NODES,1); node_closed = false(MAX_NODES,1);
steers = [-0.50, -0.32, -0.16, 0.0, 0.16, 0.32, 0.50];
wheelbase = 2.8; step_dist = 1.2; pos_res = 1.0; yaw_res = pi/8.0;
goal_tol = 2.0; yaw_tol = 0.8;
node_count = 1;
node_x(1) = sx; node_y(1) = sy; node_yaw(1) = wrap_pi_local(syaw);
node_g(1) = 0.0; node_f(1) = hypot(gx-sx, gy-sy); node_parent(1) = 0; node_open(1) = true;
goal_idx = 0;
for iter = 1:MAX_NODES
    cur = 0; best_f = 1e12;
    for i = 1:MAX_NODES, if node_open(i) && node_f(i) < best_f, best_f = node_f(i); cur = i; end; end
    if cur == 0, break; end
    node_open(cur) = false; node_closed(cur) = true;
    cx = node_x(cur); cy = node_y(cur); cyaw = node_yaw(cur);
    if hypot(gx-cx, gy-cy) < goal_tol && abs(angle_diff_local(gyaw, cyaw)) < yaw_tol
        goal_idx = cur; break;
    end
    for a = 1:N_ACTION
        delta = steers(a);
        [nx, ny, nyaw] = bicycle_step(cx, cy, cyaw, delta, step_dist, wheelbase);
        if is_collision_segment(cx, cy, nx, ny, occ_map), continue; end
        nkey_x = round(nx / pos_res); nkey_y = round(ny / pos_res); nkey_yaw = round(wrap_2pi_local(nyaw)/yaw_res);
        duplicated = false; dup_idx = 0;
        for j = 1:node_count
            jkx = round(node_x(j)/pos_res); jky = round(node_y(j)/pos_res); jkyaw = round(wrap_2pi_local(node_yaw(j))/yaw_res);
            if jkx==nkey_x && jky==nkey_y && jkyaw==nkey_yaw, duplicated=true; dup_idx=j; break; end
        end
        new_g = node_g(cur) + step_dist*(1.0 + 0.2*abs(delta));
        h = hypot(gx-nx, gy-ny) + 1.5*abs(angle_diff_local(gyaw, nyaw));
        new_f = new_g + 1.4*h;
        if duplicated
            if node_closed(dup_idx), continue; end
            if new_g >= node_g(dup_idx), continue; end
            node_x(dup_idx)=nx; node_y(dup_idx)=ny; node_yaw(dup_idx)=nyaw;
            node_g(dup_idx)=new_g; node_f(dup_idx)=new_f; node_parent(dup_idx)=cur; node_open(dup_idx)=true;
        else
            if node_count >= MAX_NODES, break; end
            node_count = node_count + 1;
            node_x(node_count)=nx; node_y(node_count)=ny; node_yaw(node_count)=nyaw;
            node_g(node_count)=new_g; node_f(node_count)=new_f; node_parent(node_count)=cur; node_open(node_count)=true;
        end
    end
end
if goal_idx == 0, return; end
tmp_x = zeros(MAX_PATH,1); tmp_y = zeros(MAX_PATH,1); tmp_yaw = zeros(MAX_PATH,1);
cnt = 0; idx = goal_idx;
while idx > 0 && cnt < MAX_PATH
    cnt = cnt + 1;
    tmp_x(cnt)=node_x(idx); tmp_y(cnt)=node_y(idx); tmp_yaw(cnt)=node_yaw(idx);
    idx = node_parent(idx);
end
for k = 1:cnt
    src = cnt - k + 1;
    path_x(k)=tmp_x(src); path_y(k)=tmp_y(src); path_yaw(k)=tmp_yaw(src);
end
path_len = cnt;
end

function [nx, ny, nyaw] = bicycle_step(x, y, yaw, steer, ds, wheelbase)
if abs(steer) < 1e-6, nx=x+ds*cos(yaw); ny=y+ds*sin(yaw); nyaw=yaw;
else, beta=ds*tan(steer)/wheelbase; radius=wheelbase/tan(steer);
    nx=x+radius*(sin(yaw+beta)-sin(yaw)); ny=y-radius*(cos(yaw+beta)-cos(yaw)); nyaw=yaw+beta;
end
nyaw = wrap_pi_local(nyaw);
end

function col = is_collision_segment(x1, y1, x2, y2, occ_map)
col = false;
for k = 0:8
    t = double(k)/8.0; x = x1 + t*(x2-x1); y = y1 + t*(y2-y1);
    if is_occupied_pose(x, y, occ_map), col = true; return; end
end
end

function occ = is_occupied_pose(x, y, occ_map)
global ROAD_Y_MAX
occ = false;
if x < 0 || x > 100 || y < -100 || y > ROAD_Y_MAX, occ = true; return; end
nr = 200; nc = 200; res_x = 1; res_y = 1; res = 1;
col = floor((x + 100)/res_x) + 1; row = floor((100 - y)/res_y) + 1;
if row < 1 || row > nr || col < 1 || col > nc, occ = true; return; end
inflate = ceil(0.8/res);
r1 = max(1, row-inflate); r2 = min(nr, row+inflate);
c1 = max(1, col-inflate); c2 = min(nc, col+inflate);
for r = r1:r2
    for c = c1:c2
        if occ_map(r, c) > 0.5, occ = true; return; end
    end
end
end

function a = wrap_pi_local(a)
while a > pi, a = a - 2*pi; end
while a < -pi, a = a + 2*pi; end
end
function a = wrap_2pi_local(a)
while a < 0, a = a + 2*pi; end
while a >= 2*pi, a = a - 2*pi; end
end
function d = angle_diff_local(a, b), d = wrap_pi_local(a - b); end
