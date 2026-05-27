function debug_day45_test_fix()
% Test the proposed fixes for Day4_5 Scenario 1:
%   - tighter yaw_tol (path ends with heading closer to goal_yaw)
%   - examine final yaw of broken path
%   - try alternative goal positions

ego_x = 0; ego_y = -20; ego_yaw = 0;
finish_point = [80; -3];
goal_yaw = 0;
traffic_size = [2.48, 11.5];

% Traffic info: T01..T07 (as currently sent to chart_45 - 7 trucks)
traffic_all_deg = [
    76.1, -5,  90;   % T00 (not sent)
    40,  -12,  90;
    40,  -24,  90;
    40,  -36,  90;
    40,  -48,  90;
    39,  -50,   0;
    51,  -50,   0;
    63,  -50,   0;
];
broken = traffic_all_deg(2:8, :);
traffic_info_broken = zeros(21, 1);
for k = 1:7
    traffic_info_broken((k-1)*3 + 1) = broken(k,1);
    traffic_info_broken((k-1)*3 + 2) = broken(k,2);
    traffic_info_broken((k-1)*3 + 3) = deg2rad(broken(k,3));
end
base_map = zeros(200, 200);
base_map(1,:)=1; base_map(end,:)=1; base_map(:,1)=1; base_map(:,end)=1;
occ_broken = add_obstacle_(base_map, traffic_info_broken, traffic_size, 7);

% Run with default yaw_tol (0.8)
[px, py, pyaw, plen] = hybrid_astar_plan(ego_x, ego_y, ego_yaw, finish_point(1), finish_point(2), goal_yaw, occ_broken, 0.8);
fprintf('DEFAULT (yaw_tol=0.8): plen=%d, final yaw=%.3f rad (%.1f deg)\n', ...
    plen, pyaw(plen), rad2deg(pyaw(plen)));

% Run with tightened yaw_tol (0.2)
[px2, py2, pyaw2, plen2] = hybrid_astar_plan(ego_x, ego_y, ego_yaw, finish_point(1), finish_point(2), goal_yaw, occ_broken, 0.2);
fprintf('TIGHT   (yaw_tol=0.2): plen=%d, final yaw=%.3f rad (%.1f deg)\n', ...
    plen2, pyaw2(plen2), rad2deg(pyaw2(plen2)));

% Now try alternative goal positions
fprintf('\n--- Trying alternative goal Y values (x=80, yaw=0) ---\n');
for goal_y = [-3, -5, -7, -10, -15, -20]
    [px3, py3, pyaw3, plen3] = hybrid_astar_plan(ego_x, ego_y, ego_yaw, 80, goal_y, 0, occ_broken, 0.3);
    if plen3 > 0
        fprintf('  goal=(80,%4d): plen=%3d, ends at (%.1f,%.1f) yaw=%.2f rad\n', ...
            goal_y, plen3, px3(plen3), py3(plen3), pyaw3(plen3));
    else
        fprintf('  goal=(80,%4d): NO PATH\n', goal_y);
    end
end

end

function y = add_obstacle_(map, traffic_info, traffic_size, n_trucks)
y = map;
veh_w = traffic_size(1); veh_l = traffic_size(2);
half_w = veh_w * 0.5 + 1.0; half_l = veh_l * 0.5 + 1.0;
x_min = -100.0; y_max = 100.0; res = 1.0; n = 200;
for i = 1:n_trucks
    idx = (i - 1) * 3;
    px = traffic_info(idx + 1); py = traffic_info(idx + 2); yaw = traffic_info(idx + 3);
    if abs(px) < 1e-6 && abs(py) < 1e-6 && abs(yaw) < 1e-6, continue; end
    c_yaw = cos(yaw); s_yaw = sin(yaw);
    for row = 1:n
        wy = y_max - (double(row) - 0.5) * res;
        for col = 1:n
            wx = x_min + (double(col) - 0.5) * res;
            dx = wx - px; dy = wy - py;
            local_x = dx*c_yaw + dy*s_yaw; local_y = -dx*s_yaw + dy*c_yaw;
            if abs(local_x) <= half_l && abs(local_y) <= half_w
                y(row, col) = 1;
            end
        end
    end
end
end

function [path_x, path_y, path_yaw, path_len] = hybrid_astar_plan(sx, sy, syaw, gx, gy, gyaw, occ_map, yaw_tol)
MAX_NODES = 4500; MAX_PATH = 220; N_ACTION = 7;
path_x = zeros(MAX_PATH,1); path_y = zeros(MAX_PATH,1); path_yaw = zeros(MAX_PATH,1); path_len = 0;
node_x = zeros(MAX_NODES,1); node_y = zeros(MAX_NODES,1); node_yaw = zeros(MAX_NODES,1);
node_g = zeros(MAX_NODES,1); node_f = zeros(MAX_NODES,1); node_parent = zeros(MAX_NODES,1);
node_open = false(MAX_NODES,1); node_closed = false(MAX_NODES,1);
steers = [-0.50, -0.32, -0.16, 0.0, 0.16, 0.32, 0.50];
wheelbase = 2.8; step_dist = 1.2; pos_res = 1.0; yaw_res = pi/8.0; goal_tol = 2.0;
node_count = 1; node_x(1)=sx; node_y(1)=sy; node_yaw(1)=wrap_pi_local(syaw);
node_g(1)=0; node_f(1)=hypot(gx-sx, gy-sy); node_parent(1)=0; node_open(1)=true;
goal_idx = 0;
for iter = 1:MAX_NODES
    cur = 0; best_f = 1e12;
    for i = 1:MAX_NODES
        if node_open(i) && node_f(i) < best_f, best_f=node_f(i); cur=i; end
    end
    if cur == 0, break; end
    node_open(cur)=false; node_closed(cur)=true;
    cx=node_x(cur); cy=node_y(cur); cyaw=node_yaw(cur);
    if hypot(gx-cx, gy-cy) < goal_tol && abs(angle_diff_local(gyaw,cyaw)) < yaw_tol
        goal_idx = cur; break;
    end
    for a = 1:N_ACTION
        delta = steers(a);
        [nx, ny, nyaw] = bicycle_step(cx, cy, cyaw, delta, step_dist, wheelbase);
        if is_collision_segment(cx, cy, nx, ny, occ_map), continue; end
        nkey_x = round(nx/pos_res); nkey_y = round(ny/pos_res); nkey_yaw = round(wrap_2pi_local(nyaw)/yaw_res);
        duplicated = false; dup_idx = 0;
        for j = 1:node_count
            if round(node_x(j)/pos_res)==nkey_x && round(node_y(j)/pos_res)==nkey_y && round(wrap_2pi_local(node_yaw(j))/yaw_res)==nkey_yaw
                duplicated=true; dup_idx=j; break;
            end
        end
        new_g = node_g(cur) + step_dist*(1 + 0.2*abs(delta));
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
            node_g(node_count)=new_g; node_f(node_count)=new_f;
            node_parent(node_count)=cur; node_open(node_count)=true;
        end
    end
end
if goal_idx == 0, return; end
tmp_x=zeros(MAX_PATH,1); tmp_y=zeros(MAX_PATH,1); tmp_yaw=zeros(MAX_PATH,1);
cnt=0; idx=goal_idx;
while idx > 0 && cnt < MAX_PATH
    cnt = cnt+1; tmp_x(cnt)=node_x(idx); tmp_y(cnt)=node_y(idx); tmp_yaw(cnt)=node_yaw(idx);
    idx = node_parent(idx);
end
for k=1:cnt
    src=cnt-k+1; path_x(k)=tmp_x(src); path_y(k)=tmp_y(src); path_yaw(k)=tmp_yaw(src);
end
path_len = cnt;
end

function [nx,ny,nyaw] = bicycle_step(x,y,yaw,steer,ds,wheelbase)
if abs(steer)<1e-6
    nx=x+ds*cos(yaw); ny=y+ds*sin(yaw); nyaw=yaw;
else
    beta=ds*tan(steer)/wheelbase; radius=wheelbase/tan(steer);
    nx=x+radius*(sin(yaw+beta)-sin(yaw)); ny=y-radius*(cos(yaw+beta)-cos(yaw)); nyaw=yaw+beta;
end
nyaw = wrap_pi_local(nyaw);
end

function col = is_collision_segment(x1,y1,x2,y2,occ_map)
col=false;
for k=0:8
    t=double(k)/8; x=x1+t*(x2-x1); y=y1+t*(y2-y1);
    if is_occupied_pose(x,y,occ_map), col=true; return; end
end
end

function occ = is_occupied_pose(x,y,occ_map)
occ=false;
if x < 0 || x > 100 || y < -100 || y > -2, occ=true; return; end
nr=200; nc=200; res_x=1; res_y=1; res=1;
col=floor((x+100)/res_x)+1; row=floor((100-y)/res_y)+1;
if row<1||row>nr||col<1||col>nc, occ=true; return; end
inflate=ceil(0.8/res);
r1=max(1,row-inflate); r2=min(nr,row+inflate); c1=max(1,col-inflate); c2=min(nc,col+inflate);
for r=r1:r2, for c=c1:c2, if occ_map(r,c)>0.5, occ=true; return; end; end; end
end

function a=wrap_pi_local(a)
while a>pi, a=a-2*pi; end
while a<-pi, a=a+2*pi; end
end
function a=wrap_2pi_local(a)
while a<0, a=a+2*pi; end
while a>=2*pi, a=a-2*pi; end
end
function d=angle_diff_local(a,b), d=wrap_pi_local(a-b); end
