function debug_day45_after_boundary_fix2()
% Same as before but using the planner that already works (with global road_y_max)
global ROAD_Y_MAX
ROAD_Y_MAX = 0;  % Updated boundary

traffic_all_deg = [76.1 -5 90; 40 -12 90; 40 -24 90; 40 -36 90; 40 -48 90; 39 -50 0; 51 -50 0; 63 -50 0];
ti = zeros(21,1);
for k=1:7
    ti((k-1)*3+1)=traffic_all_deg(k,1); ti((k-1)*3+2)=traffic_all_deg(k,2); ti((k-1)*3+3)=deg2rad(traffic_all_deg(k,3));
end

% New chart_45 logic: off-road marking + trucks
y = zeros(200,200);
res = 1.0; x_min = -100.0; y_max = 100.0; n = 200;

for row = 1:n
    wy = y_max - (row - 0.5) * res;
    for col = 1:n
        wx = x_min + (col - 0.5) * res;
        if wx < -1.5 || wx > 101.5 || wy < -101.5 || wy > 1.5
            y(row,col) = 1;
        end
    end
end

hw=2.48*0.5+1; hl=11.5*0.5+1;
for i=1:7
    idx=(i-1)*3; px=ti(idx+1); py=ti(idx+2); yaw=ti(idx+3);
    c=cos(yaw); s=sin(yaw);
    for row=1:200, wy=100-(row-0.5);
        for col=1:200, wx=-100+(col-0.5);
            dx=wx-px; dy=wy-py; lx=dx*c+dy*s; ly=-dx*s+dy*c;
            if abs(lx)<=hl && abs(ly)<=hw, y(row,col)=1; end
        end
    end
end

fprintf('Obstacle cells: %d (of 40000)\n', sum(y(:)>0.5));

% Use the proven planner from debug_day45_boundary2 (uses global ROAD_Y_MAX)
[px, py, ~, plen] = hybrid_astar_plan(0, -20, 0, 80, -3, 0, y);
if plen > 0
    fprintf('AFTER FIX: plen=%d, ends (%.2f,%.2f), y range [%.2f, %.2f]\n', ...
        plen, px(plen), py(plen), min(py(1:plen)), max(py(1:plen)));
else
    fprintf('NO PATH after fix\n');
end

figure('Position',[100 100 900 900]);
hold on; grid on; axis equal;
imagesc([-100 100], [100 -100], y); colormap(flipud(gray)); axis xy;
xlim([-5 105]); ylim([-105 5]);
if plen > 0
    plot(px(1:plen), py(1:plen), 'm.-', 'LineWidth',2);
end
plot(0,-20,'go','MarkerSize',12,'LineWidth',2);
plot(80,-3,'r*','MarkerSize',16,'LineWidth',2);
title({'After boundary fix: black = off-road or truck','road\_y\_max=0; off-road marking pushed 1.5m out'});
saveas(gcf, fullfile(fileparts(mfilename('fullpath')), 'debug_day45_after_boundary.png'));

end

%% Planner (same as debug_day45_boundary2.m, uses global ROAD_Y_MAX)
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
    cnt = cnt + 1; tmp_x(cnt)=node_x(idx); tmp_y(cnt)=node_y(idx); tmp_yaw(cnt)=node_yaw(idx);
    idx = node_parent(idx);
end
for k = 1:cnt, src = cnt - k + 1; path_x(k)=tmp_x(src); path_y(k)=tmp_y(src); path_yaw(k)=tmp_yaw(src); end
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
