function debug_day45_t00_actual_size()
global ROAD_Y_MAX
ROAD_Y_MAX = 0;

% T01..T07 as trucks (11.5 × 2.48), T00 as a car (4.47 × 1.97)
trucks = [40 -12 90; 40 -24 90; 40 -36 90; 40 -48 90; 39 -50 0; 51 -50 0; 63 -50 0];
t00 = [76.1 -5 90];

y = zeros(200,200);
y(1,:)=1; y(end,:)=1; y(:,1)=1; y(:,end)=1;

% Trucks
hw=2.48*0.5+1; hl=11.5*0.5+1;
for i=1:7
    px=trucks(i,1); py=trucks(i,2); yaw=deg2rad(trucks(i,3));
    y = stamp_box(y, px, py, yaw, hl, hw);
end
% T00 (car-sized)
hw0=1.97*0.5+1; hl0=4.47*0.5+1;
y = stamp_box(y, t00(1), t00(2), deg2rad(t00(3)), hl0, hw0);

fprintf('Obstacle cells: %d\n', sum(y(:)>0.5));
[px, py, ~, plen] = hybrid_astar_plan(0,-20,0, 80,-3,0, y);
if plen > 0
    fprintf('plen=%d, ends (%.2f,%.2f)\n', plen, px(plen), py(plen));
else
    fprintf('NO PATH\n');
end

figure('Position',[100 100 1000 900]);
hold on; grid on; axis equal;
imagesc([-100 100], [100 -100], y); colormap(flipud(gray)); axis xy;
xlim([-5 105]); ylim([-25 5]);
if plen > 0
    plot(px(1:plen), py(1:plen), 'm.-', 'LineWidth',2);
end
plot(0,-20,'go','MarkerSize',12,'LineWidth',2);
plot(80,-3,'r*','MarkerSize',16,'LineWidth',2);
plot([0 100],[0 0],'g--');
title('T00 with car-size (4.47\times1.97), trucks T01..T07 with truck-size');
saveas(gcf, fullfile(fileparts(mfilename('fullpath')), 'debug_day45_t00_actual.png'));
end

function y = stamp_box(y, px, py, yaw, hl, hw)
c=cos(yaw); s=sin(yaw);
for row=1:200, wy=100-(row-0.5);
    for col=1:200, wx=-100+(col-0.5);
        dx=wx-px; dy=wy-py; lx=dx*c+dy*s; ly=-dx*s+dy*c;
        if abs(lx)<=hl && abs(ly)<=hw, y(row,col)=1; end
    end
end
end

function [path_x, path_y, path_yaw, path_len] = hybrid_astar_plan(sx, sy, syaw, gx, gy, gyaw, occ_map)
MAX_NODES = 4500; MAX_PATH = 220;
path_x = zeros(MAX_PATH,1); path_y = zeros(MAX_PATH,1); path_yaw = zeros(MAX_PATH,1); path_len = 0;
node_x = zeros(MAX_NODES,1); node_y = zeros(MAX_NODES,1); node_yaw = zeros(MAX_NODES,1);
node_g = zeros(MAX_NODES,1); node_f = zeros(MAX_NODES,1); node_parent = zeros(MAX_NODES,1);
node_open = false(MAX_NODES,1); node_closed = false(MAX_NODES,1);
steers = [-0.50, -0.32, -0.16, 0.0, 0.16, 0.32, 0.50];
L = 2.8; ds = 1.2; pres = 1.0; yres = pi/8.0;
nc = 1; node_x(1)=sx; node_y(1)=sy; node_yaw(1)=wp_(syaw); node_f(1)=hypot(gx-sx,gy-sy); node_open(1)=true;
gid = 0;
for it = 1:MAX_NODES
    cur = 0; bf = 1e12;
    for i = 1:MAX_NODES, if node_open(i) && node_f(i) < bf, bf=node_f(i); cur=i; end; end
    if cur == 0, break; end
    node_open(cur)=false; node_closed(cur)=true;
    cx=node_x(cur); cy=node_y(cur); cyaw=node_yaw(cur);
    if hypot(gx-cx,gy-cy)<2 && abs(adf(gyaw,cyaw))<0.8, gid=cur; break; end
    for a = 1:7
        d = steers(a);
        if abs(d)<1e-6, nxv=cx+ds*cos(cyaw); nyv=cy+ds*sin(cyaw); nyawv=cyaw;
        else, beta=ds*tan(d)/L; r=L/tan(d); nxv=cx+r*(sin(cyaw+beta)-sin(cyaw)); nyv=cy-r*(cos(cyaw+beta)-cos(cyaw)); nyawv=cyaw+beta; end
        nyawv = wp_(nyawv);
        if iscol_(cx,cy,nxv,nyv,occ_map), continue; end
        kx=round(nxv/pres); ky=round(nyv/pres); kyaw=round(w2p_(nyawv)/yres);
        dup=false; di=0;
        for j=1:nc
            if round(node_x(j)/pres)==kx && round(node_y(j)/pres)==ky && round(w2p_(node_yaw(j))/yres)==kyaw
                dup=true; di=j; break; end
        end
        newg = node_g(cur) + ds*(1 + 0.2*abs(d));
        h = hypot(gx-nxv,gy-nyv) + 1.5*abs(adf(gyaw,nyawv));
        newf = newg + 1.4*h;
        if dup
            if node_closed(di), continue; end
            if newg >= node_g(di), continue; end
            node_x(di)=nxv; node_y(di)=nyv; node_yaw(di)=nyawv;
            node_g(di)=newg; node_f(di)=newf; node_parent(di)=cur; node_open(di)=true;
        else
            if nc >= MAX_NODES, break; end
            nc=nc+1;
            node_x(nc)=nxv; node_y(nc)=nyv; node_yaw(nc)=nyawv;
            node_g(nc)=newg; node_f(nc)=newf; node_parent(nc)=cur; node_open(nc)=true;
        end
    end
end
if gid == 0, return; end
tx=zeros(MAX_PATH,1); ty=zeros(MAX_PATH,1); tyaw=zeros(MAX_PATH,1);
cnt=0; idx=gid;
while idx>0 && cnt<MAX_PATH, cnt=cnt+1; tx(cnt)=node_x(idx); ty(cnt)=node_y(idx); tyaw(cnt)=node_yaw(idx); idx=node_parent(idx); end
for k=1:cnt, s=cnt-k+1; path_x(k)=tx(s); path_y(k)=ty(s); path_yaw(k)=tyaw(s); end
path_len = cnt;
end

function col = iscol_(x1,y1,x2,y2,m)
col=false;
for k=0:8, t=k/8; x=x1+t*(x2-x1); y=y1+t*(y2-y1);
    if iop_(x,y,m), col=true; return; end
end
end
function occ = iop_(x,y,m)
global ROAD_Y_MAX
occ=false;
if x<0||x>100||y<-100||y>ROAD_Y_MAX, occ=true; return; end
r=floor(100-y)+1; c=floor(x+100)+1;
if r<1||r>200||c<1||c>200, occ=true; return; end
for rr=max(1,r-1):min(200,r+1)
    for cc=max(1,c-1):min(200,c+1)
        if m(rr,cc)>0.5, occ=true; return; end
    end
end
end
function a=wp_(a), while a>pi, a=a-2*pi; end; while a<-pi, a=a+2*pi; end; end
function a=w2p_(a), while a<0, a=a+2*pi; end; while a>=2*pi, a=a-2*pi; end; end
function d=adf(a,b), d=wp_(a-b); end
