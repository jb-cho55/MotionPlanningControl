function debug_day45_fix_test()
% Test fixes for Day4_5 Scenario 1 in closed loop:
%   FIX 1: dist_goal stop threshold 0.8 -> 2.0 (matches goal_tol)
%   FIX 2: tick replan threshold 15 -> 200 (replan only on demand)
%   FIX 3: when replan fails, keep previous path instead of straight-line fallback

ego0 = [0, -20, 0, 0];
finish_point = [80; -3]; goal_yaw = 0;
traffic_size = [2.48, 11.5];
traffic_all_deg = [76.1 -5 90; 40 -12 90; 40 -24 90; 40 -36 90; 40 -48 90; 39 -50 0; 51 -50 0; 63 -50 0];
broken_t = traffic_all_deg(2:8, :);
traffic_info = zeros(21,1);
for k=1:7
    traffic_info((k-1)*3+1)=broken_t(k,1); traffic_info((k-1)*3+2)=broken_t(k,2); traffic_info((k-1)*3+3)=deg2rad(broken_t(k,3));
end
base_map = zeros(200,200); base_map(1,:)=1; base_map(end,:)=1; base_map(:,1)=1; base_map(:,end)=1;
occ = add_obstacle_(base_map, traffic_info, traffic_size, 7);

% Simulate WITH FIXES
ego = ego0; dt=0.1; T=40; N=round(T/dt);
xs=zeros(1,N); ys=zeros(1,N); yaws=zeros(1,N);
px=zeros(220,1); py=zeros(220,1); pyaw=zeros(220,1); plen=0;
last_goal=[1e9 1e9]; tick=1000;

for k=1:N
    xs(k)=ego(1); ys(k)=ego(2); yaws(k)=ego(3);

    tick = tick + 1;
    need_replan = (plen < 2) || any(abs(finish_point' - last_goal) > 0.2);  % FIX 2: removed tick > 15
    if tick > 200, need_replan = true; end  % FIX 2: much less frequent
    if need_replan
        [px2, py2, pyaw2, plen2] = hybrid_astar_plan(ego(1), ego(2), ego(3), finish_point(1), finish_point(2), goal_yaw, occ);
        if plen2 >= 2
            px=px2; py=py2; pyaw=pyaw2; plen=plen2;
        end
        % FIX 3: do NOT clobber path with straight-line fallback if replan fails; keep previous
        tick = 0; last_goal = finish_point';
    end

    [desired_ax, steer_cmd] = follow_path_controller(ego(1), ego(2), ego(3), ego(4), ...
        finish_point(1), finish_point(2), goal_yaw, px, py, plen);

    L=2.8;
    ego(4) = ego(4) + desired_ax*dt; if ego(4)<0, ego(4)=0; end
    ego(1) = ego(1) + ego(4)*cos(ego(3))*dt;
    ego(2) = ego(2) + ego(4)*sin(ego(3))*dt;
    ego(3) = ego(3) + (ego(4)/L)*tan(steer_cmd)*dt;
end

figure('Position',[100 100 1100 500]);
hold on; grid on; axis equal;
imagesc([-100 100], [100 -100], occ); colormap(flipud(gray)); axis xy;
xlim([-5 110]); ylim([-25 5]);
plot(xs, ys, 'r-', 'LineWidth', 2, 'DisplayName','vehicle');
plot(px(1:plen), py(1:plen), 'b.-', 'DisplayName','final path');
plot(ego0(1), ego0(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(finish_point(1), finish_point(2), 'r*', 'MarkerSize', 14);
plot([0 100 100 0 0], [-2 -2 -100 -100 -2], 'g--', 'DisplayName','planner boundary');
legend('Location','best');
title(sprintf('FIXED closed-loop: end (%.1f,%.1f) yaw=%.2frad v=%.1fm/s', ego(1), ego(2), ego(3), ego(4)));
saveas(gcf, fullfile(fileparts(mfilename('fullpath')), 'debug_day45_fixed.png'));

fprintf('FIXED end pose: (%.2f, %.2f), yaw=%.3f, v=%.2f\n', ego(1), ego(2), ego(3), ego(4));
fprintf('FIXED final dist to goal: %.2fm\n', hypot(finish_point(1)-ego(1), finish_point(2)-ego(2)));
end

function y = add_obstacle_(map, ti, ts, n_trucks)
y=map; w=ts(1); l=ts(2); hw=w*0.5+1; hl=l*0.5+1;
for i=1:n_trucks
    idx=(i-1)*3; px=ti(idx+1); py=ti(idx+2); yaw=ti(idx+3);
    if abs(px)<1e-6&&abs(py)<1e-6&&abs(yaw)<1e-6, continue; end
    c=cos(yaw); s=sin(yaw);
    for r=1:200, wy=100-(r-0.5);
        for col=1:200, wx=-100+(col-0.5);
            dx=wx-px; dy=wy-py; lx=dx*c+dy*s; ly=-dx*s+dy*c;
            if abs(lx)<=hl && abs(ly)<=hw, y(r,col)=1; end
        end
    end
end
end

function [px, py, pyaw, plen] = hybrid_astar_plan(sx, sy, syaw, gx, gy, gyaw, occ_map)
MAX_NODES=4500; MAX_PATH=220;
px=zeros(MAX_PATH,1); py=zeros(MAX_PATH,1); pyaw=zeros(MAX_PATH,1); plen=0;
nx_=zeros(MAX_NODES,1); ny_=zeros(MAX_NODES,1); nyaw_=zeros(MAX_NODES,1);
ng=zeros(MAX_NODES,1); nf=zeros(MAX_NODES,1); npar=zeros(MAX_NODES,1);
nopen=false(MAX_NODES,1); nclosed=false(MAX_NODES,1);
steers=[-0.5,-0.32,-0.16,0,0.16,0.32,0.5]; L=2.8; ds=1.2;
nc=1; nx_(1)=sx; ny_(1)=sy; nyaw_(1)=wp(syaw); nf(1)=hypot(gx-sx,gy-sy); nopen(1)=true;
gid=0;
for it=1:MAX_NODES
    cur=0; bf=1e12;
    for i=1:MAX_NODES, if nopen(i)&&nf(i)<bf, bf=nf(i); cur=i; end; end
    if cur==0, break; end
    nopen(cur)=false; nclosed(cur)=true;
    cx=nx_(cur); cy=ny_(cur); cyaw=nyaw_(cur);
    if hypot(gx-cx,gy-cy)<2 && abs(ad(gyaw,cyaw))<0.8, gid=cur; break; end
    for a=1:7
        d=steers(a);
        if abs(d)<1e-6, nxv=cx+ds*cos(cyaw); nyv=cy+ds*sin(cyaw); nyawv=cyaw;
        else, beta=ds*tan(d)/L; r=L/tan(d); nxv=cx+r*(sin(cyaw+beta)-sin(cyaw)); nyv=cy-r*(cos(cyaw+beta)-cos(cyaw)); nyawv=cyaw+beta; end
        nyawv=wp(nyawv);
        if cs(cx,cy,nxv,nyv,occ_map), continue; end
        kx=round(nxv); ky=round(nyv); kyaw=round(w2p(nyawv)/(pi/8));
        dup=false; di=0;
        for j=1:nc
            if round(nx_(j))==kx && round(ny_(j))==ky && round(w2p(nyaw_(j))/(pi/8))==kyaw, dup=true; di=j; break; end
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
% FIX 1: stop threshold widened 0.8 -> 2.0 (matches goal_tol)
if dg<2.0
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

function c=cs(x1,y1,x2,y2,m), c=false; for k=0:8, t=k/8; if iop(x1+t*(x2-x1),y1+t*(y2-y1),m), c=true; return; end; end; end
function o=iop(x,y,m), o=false; if x<0||x>100||y<-100||y>-2, o=true; return; end
r=floor(100-y)+1; c=floor(x+100)+1; if r<1||r>200||c<1||c>200, o=true; return; end
for rr=max(1,r-1):min(200,r+1), for cc=max(1,c-1):min(200,c+1), if m(rr,cc)>0.5, o=true; return; end; end; end
end
function a=wp(a), while a>pi, a=a-2*pi; end; while a<-pi, a=a+2*pi; end; end
function a=w2p(a), while a<0, a=a+2*pi; end; while a>=2*pi, a=a-2*pi; end; end
function d=ad(a,b), d=wp(a-b); end
function y=sat(x,lo,hi), y=x; if y<lo, y=lo; elseif y>hi, y=hi; end; end
