function debug_day45_visualize()
% Visualize what happens to the controlled vehicle after the path ends.

ego0 = [0, -20, 0, 0];  % x, y, yaw, v
finish_point = [80; -3];
goal_yaw = 0;
traffic_size = [2.48, 11.5];

traffic_all_deg = [
    76.1, -5,  90;  40, -12, 90;  40, -24, 90;  40, -36, 90;
    40,  -48, 90;  39, -50,  0;  51, -50,  0;  63, -50, 0;
];
broken_t = traffic_all_deg(2:8, :);
traffic_info = zeros(21, 1);
for k = 1:7
    traffic_info((k-1)*3 + 1) = broken_t(k,1);
    traffic_info((k-1)*3 + 2) = broken_t(k,2);
    traffic_info((k-1)*3 + 3) = deg2rad(broken_t(k,3));
end
base_map = zeros(200, 200);
base_map(1,:)=1; base_map(end,:)=1; base_map(:,1)=1; base_map(:,end)=1;
occ = add_obstacle_(base_map, traffic_info, traffic_size, 7);

% Plan from start
[px, py, pyaw, plen] = hybrid_astar_plan(ego0(1), ego0(2), ego0(3), finish_point(1), finish_point(2), goal_yaw, occ);
fprintf('Initial plan: plen=%d, ends at (%.2f,%.2f) yaw=%.3f\n', plen, px(plen), py(plen), pyaw(plen));

% Simulate vehicle with pure-pursuit + replan every 15 ticks
% Use SAME control law as chart_18 to faithfully reproduce.
ego = ego0;
dt = 0.1;
T = 60;
N = round(T/dt);
xs = zeros(1, N); ys = zeros(1, N); yaws = zeros(1, N);
last_goal = [1e9, 1e9];
tick = 1000;
for k = 1:N
    xs(k) = ego(1); ys(k) = ego(2); yaws(k) = ego(3);

    % Track replan logic
    tick = tick + 1;
    need_replan = (plen < 2) || (tick > 15) || any(abs(finish_point' - last_goal) > 0.2);
    if need_replan
        [px2, py2, pyaw2, plen2] = hybrid_astar_plan(ego(1), ego(2), ego(3), finish_point(1), finish_point(2), goal_yaw, occ);
        if plen2 >= 2
            px = px2; py = py2; pyaw = pyaw2; plen = plen2;
        else
            % straight-line fallback
            px(:) = 0; py(:) = 0; pyaw(:) = 0;
            px(1) = ego(1); py(1) = ego(2); pyaw(1) = ego(3);
            px(2) = finish_point(1); py(2) = finish_point(2); pyaw(2) = goal_yaw;
            plen = 2;
            fprintf('  Replan at t=%.1f failed → straight-line fallback\n', k*dt);
        end
        tick = 0;
        last_goal = finish_point';
    end

    [desired_ax, steer_cmd] = follow_path_controller(ego(1), ego(2), ego(3), ego(4), ...
        finish_point(1), finish_point(2), goal_yaw, px, py, plen);

    % Forward Euler bicycle model integration
    L = 2.8;
    ego(4) = ego(4) + desired_ax * dt;
    if ego(4) < 0, ego(4) = 0; end
    ego(1) = ego(1) + ego(4) * cos(ego(3)) * dt;
    ego(2) = ego(2) + ego(4) * sin(ego(3)) * dt;
    ego(3) = ego(3) + (ego(4)/L) * tan(steer_cmd) * dt;
end

% Plot
figure('Position',[100 100 1000 600]);
hold on; axis equal; grid on;
imagesc([-100 100], [100 -100], occ); colormap(flipud(gray)); axis xy;
xlim([-5 110]); ylim([-15 15]);
plot(px(1:plen), py(1:plen), 'b.-', 'DisplayName','final planned path');
plot(xs(1:k), ys(1:k), 'r-', 'LineWidth', 1.5, 'DisplayName','vehicle trajectory');
plot(ego0(1), ego0(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(finish_point(1), finish_point(2), 'r*', 'MarkerSize', 14);
% Draw the planner's drivable boundary
plot([0 100 100 0 0], [-2 -2 -100 -100 -2], 'g--', 'DisplayName','planner road boundary y=-2');
legend('Location','best');
title(sprintf('Closed-loop simulation, end pose=(%.2f,%.2f) yaw=%.2frad', ego(1), ego(2), ego(3)));
saveas(gcf, fullfile(fileparts(mfilename('fullpath')), 'debug_day45_closedloop.png'));
fprintf('End pose: (%.2f, %.2f), yaw=%.3f rad (%.1f deg), v=%.2f m/s\n', ego(1), ego(2), ego(3), rad2deg(ego(3)), ego(4));
fprintf('Final dist to goal: %.2fm\n', hypot(finish_point(1)-ego(1), finish_point(2)-ego(2)));
end

function y = add_obstacle_(map, traffic_info, traffic_size, n_trucks)
y = map; veh_w = traffic_size(1); veh_l = traffic_size(2);
half_w = veh_w * 0.5 + 1.0; half_l = veh_l * 0.5 + 1.0;
for i = 1:n_trucks
    idx = (i - 1) * 3;
    px = traffic_info(idx + 1); py = traffic_info(idx + 2); yaw = traffic_info(idx + 3);
    if abs(px) < 1e-6 && abs(py) < 1e-6 && abs(yaw) < 1e-6, continue; end
    c_yaw = cos(yaw); s_yaw = sin(yaw);
    for row = 1:200
        wy = 100 - (row - 0.5);
        for col = 1:200
            wx = -100 + (col - 0.5);
            dx = wx - px; dy = wy - py;
            lx = dx*c_yaw + dy*s_yaw; ly = -dx*s_yaw + dy*c_yaw;
            if abs(lx) <= half_l && abs(ly) <= half_w, y(row,col)=1; end
        end
    end
end
end

function [px, py, pyaw, plen] = hybrid_astar_plan(sx, sy, syaw, gx, gy, gyaw, occ_map)
MAX_NODES = 4500; MAX_PATH = 220; N_ACTION = 7;
px=zeros(MAX_PATH,1); py=zeros(MAX_PATH,1); pyaw=zeros(MAX_PATH,1); plen=0;
nx_=zeros(MAX_NODES,1); ny_=zeros(MAX_NODES,1); nyaw_=zeros(MAX_NODES,1);
ng=zeros(MAX_NODES,1); nf=zeros(MAX_NODES,1); npar=zeros(MAX_NODES,1);
nopen=false(MAX_NODES,1); nclosed=false(MAX_NODES,1);
steers=[-0.5,-0.32,-0.16,0,0.16,0.32,0.5]; L=2.8; ds=1.2; pres=1; yres=pi/8; gtol=2; ytol=0.8;
nc=1; nx_(1)=sx; ny_(1)=sy; nyaw_(1)=wp(syaw); nf(1)=hypot(gx-sx,gy-sy); nopen(1)=true;
gid=0;
for it=1:MAX_NODES
    cur=0; bf=1e12;
    for i=1:MAX_NODES, if nopen(i)&&nf(i)<bf, bf=nf(i); cur=i; end; end
    if cur==0, break; end
    nopen(cur)=false; nclosed(cur)=true;
    cx=nx_(cur); cy=ny_(cur); cyaw=nyaw_(cur);
    if hypot(gx-cx,gy-cy)<gtol && abs(ad(gyaw,cyaw))<ytol, gid=cur; break; end
    for a=1:N_ACTION
        d=steers(a); [nxv,nyv,nyawv]=bs(cx,cy,cyaw,d,ds,L);
        if cs(cx,cy,nxv,nyv,occ_map), continue; end
        kx=round(nxv/pres); ky=round(nyv/pres); kyaw=round(w2p(nyawv)/yres);
        dup=false; di=0;
        for j=1:nc
            if round(nx_(j)/pres)==kx && round(ny_(j)/pres)==ky && round(w2p(nyaw_(j))/yres)==kyaw, dup=true; di=j; break; end
        end
        newg=ng(cur)+ds*(1+0.2*abs(d));
        h=hypot(gx-nxv,gy-nyv)+1.5*abs(ad(gyaw,nyawv)); newf=newg+1.4*h;
        if dup
            if nclosed(di), continue; end
            if newg>=ng(di), continue; end
            nx_(di)=nxv; ny_(di)=nyv; nyaw_(di)=nyawv; ng(di)=newg; nf(di)=newf; npar(di)=cur; nopen(di)=true;
        else
            if nc>=MAX_NODES, break; end
            nc=nc+1; nx_(nc)=nxv; ny_(nc)=nyv; nyaw_(nc)=nyawv; ng(nc)=newg; nf(nc)=newf; npar(nc)=cur; nopen(nc)=true;
        end
    end
end
if gid==0, return; end
tx=zeros(MAX_PATH,1); ty=zeros(MAX_PATH,1); tyaw=zeros(MAX_PATH,1);
cnt=0; idx=gid;
while idx>0 && cnt<MAX_PATH, cnt=cnt+1; tx(cnt)=nx_(idx); ty(cnt)=ny_(idx); tyaw(cnt)=nyaw_(idx); idx=npar(idx); end
for k=1:cnt, s=cnt-k+1; px(k)=tx(s); py(k)=ty(s); pyaw(k)=tyaw(s); end
plen=cnt;
end

function [desired_ax, steer_cmd] = follow_path_controller(ego_x, ego_y, ego_yaw, ego_v, gx, gy, gyaw, px, py, plen)
L=2.8; ms=0.45;
dg=hypot(gx-ego_x,gy-ego_y); yeg=ad(gyaw,ego_yaw);
if dg<0.8
    dv=0; steer_cmd=sat(0.8*yeg,-ms,ms); desired_ax=sat(1.2*(dv-ego_v),-3,0.8); return;
end
near=1; bd=1e12;
for i=1:220, if i<=plen, d=(px(i)-ego_x)^2+(py(i)-ego_y)^2; if d<bd, bd=d; near=i; end; end; end
la=2+0.25*abs(ego_v); if la>4.5, la=4.5; end
tgt=near; ac=0;
for i=near:219, if i>=plen, break; end
    seg=hypot(px(i+1)-px(i),py(i+1)-py(i)); ac=ac+seg; tgt=i+1; if ac>=la, break; end
end
tx=px(tgt); ty=py(tgt); dx=tx-ego_x; dy=ty-ego_y;
c=cos(ego_yaw); s=sin(ego_yaw); lx=c*dx+s*dy; ly=-s*dx+c*dy;
if lx<0.2, lx=0.2; end
steer_cmd=atan2(2*L*ly,la^2); steer_cmd=sat(steer_cmd,-ms,ms);
if dg>15, dv=2.5; elseif dg>6, dv=1.6; else, dv=0.8; end
desired_ax=0.8*(dv-ego_v); desired_ax=sat(desired_ax,-3,1.5);
end

function [nx,ny,nyaw]=bs(x,y,yaw,steer,ds,L)
if abs(steer)<1e-6, nx=x+ds*cos(yaw); ny=y+ds*sin(yaw); nyaw=yaw;
else, beta=ds*tan(steer)/L; r=L/tan(steer); nx=x+r*(sin(yaw+beta)-sin(yaw)); ny=y-r*(cos(yaw+beta)-cos(yaw)); nyaw=yaw+beta; end
nyaw=wp(nyaw);
end
function c=cs(x1,y1,x2,y2,m), c=false; for k=0:8, t=k/8; if iop(x1+t*(x2-x1),y1+t*(y2-y1),m), c=true; return; end; end; end
function o=iop(x,y,m), o=false; if x<0||x>100||y<-100||y>-2, o=true; return; end
r=floor(100-y)+1; c=floor(x+100)+1; if r<1||r>200||c<1||c>200, o=true; return; end
inf=1; r1=max(1,r-inf); r2=min(200,r+inf); c1=max(1,c-inf); c2=min(200,c+inf);
for rr=r1:r2, for cc=c1:c2, if m(rr,cc)>0.5, o=true; return; end; end; end
end
function a=wp(a), while a>pi, a=a-2*pi; end; while a<-pi, a=a+2*pi; end; end
function a=w2p(a), while a<0, a=a+2*pi; end; while a>=2*pi, a=a-2*pi; end; end
function d=ad(a,b), d=wp(a-b); end
function y=sat(x,lo,hi), y=x; if y<lo, y=lo; elseif y>hi, y=hi; end; end
