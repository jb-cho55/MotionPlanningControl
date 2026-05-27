function debug_day45_after_boundary_fix()
% Visualize the obstacle map AFTER the boundary fix:
%   - chart_45 marks off-road cells as obstacles
%   - chart_18 road_y_max = 0 (matches LaneR.0 north edge)

traffic_all_deg = [76.1 -5 90; 40 -12 90; 40 -24 90; 40 -36 90; 40 -48 90; 39 -50 0; 51 -50 0; 63 -50 0];
ti = zeros(21,1);
for k=1:7
    ti((k-1)*3+1)=traffic_all_deg(k,1); ti((k-1)*3+2)=traffic_all_deg(k,2); ti((k-1)*3+3)=deg2rad(traffic_all_deg(k,3));
end

% New chart_45: marks off-road cells AND trucks
y = zeros(200,200);
res = 1.0; x_min = -100.0; y_max = 100.0; n = 200;

% Off-road marking (the boundary) — pushed 1.5m outside parking lot so the
% planner's 0.8m inflation in is_occupied_pose doesn't bleed back inside.
for row = 1:n
    wy = y_max - (row - 0.5) * res;
    for col = 1:n
        wx = x_min + (col - 0.5) * res;
        if wx < -1.5 || wx > 101.5 || wy < -101.5 || wy > 1.5
            y(row,col) = 1;
        end
    end
end

% Trucks
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

% Quick sanity check: is the start point free?
sx=0; sy=-20;
r0=floor(100-sy)+1; c0=floor(sx+100)+1;
fprintf('Start cell (row=%d, col=%d), occ=%g\n', r0, c0, y(r0,c0));
for rr=r0-1:r0+1, for cc=c0-1:c0+1, fprintf('  (%d,%d)=%g\n', rr, cc, y(rr,cc)); end; end
fprintf('Cells around col=99,100 (wx=-1.5,-0.5):\n');
fprintf('  col 98 (wx=-2.5): row 121 = %g\n', y(121,98));
fprintf('  col 99 (wx=-1.5): row 121 = %g\n', y(121,99));
fprintf('  col 100 (wx=-0.5): row 121 = %g\n', y(121,100));
fprintf('Cells along ego path (row=121, cols 101-105):\n');
for cc=101:105, fprintf('  col %d (wx=%.1f): %g\n', cc, cc-100.5, y(121,cc)); end

% Run the planner with road_y_max=0
[px, py, ~, plen] = run_plan(0,-20,0, 80,-3,0, y);
if plen > 0
    fprintf('road_y_max=0 with boundary visualized: plen=%d, ends (%.2f,%.2f)\n', plen, px(plen), py(plen));
else
    fprintf('NO PATH\n');
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
title({'After boundary fix: black = off-road OR truck','road\_y\_max=0 (\equiv LaneR.0 north edge)'});
saveas(gcf, fullfile(fileparts(mfilename('fullpath')), 'debug_day45_after_boundary.png'));
end

function [px, py, pyaw, plen] = run_plan(sx, sy, syaw, gx, gy, gyaw, occ)
MAX_NODES=4500; MAX_PATH=220;
px=zeros(MAX_PATH,1); py=zeros(MAX_PATH,1); pyaw=zeros(MAX_PATH,1); plen=0;
nx_=zeros(MAX_NODES,1); ny_=zeros(MAX_NODES,1); nyaw_=zeros(MAX_NODES,1);
ng=zeros(MAX_NODES,1); nf=zeros(MAX_NODES,1); npar=zeros(MAX_NODES,1);
nopen=false(MAX_NODES,1); nclosed=false(MAX_NODES,1);
steers=[-0.5,-0.32,-0.16,0,0.16,0.32,0.5]; L=2.8; ds=1.2;
nc=1; nx_(1)=sx; ny_(1)=sy; nyaw_(1)=wp_(syaw); nf(1)=hypot(gx-sx,gy-sy); nopen(1)=true;
gid=0;
for it=1:MAX_NODES
    cur=0; bf=1e12;
    for i=1:MAX_NODES, if nopen(i)&&nf(i)<bf, bf=nf(i); cur=i; end; end
    if cur==0, break; end
    nopen(cur)=false; nclosed(cur)=true;
    cx=nx_(cur); cy=ny_(cur); cyaw=nyaw_(cur);
    if hypot(gx-cx,gy-cy)<2 && abs(adf(gyaw,cyaw))<0.8, gid=cur; break; end
    for a=1:7
        d=steers(a);
        if abs(d)<1e-6, nxv=cx+ds*cos(cyaw); nyv=cy+ds*sin(cyaw); nyawv=cyaw;
        else, beta=ds*tan(d)/L; r=L/tan(d); nxv=cx+r*(sin(cyaw+beta)-sin(cyaw)); nyv=cy-r*(cos(cyaw+beta)-cos(cyaw)); nyawv=cyaw+beta; end
        nyawv=wp_(nyawv);
        if iscol(cx,cy,nxv,nyv,occ), continue; end
        kx=round(nxv); ky=round(nyv); kyaw=round(w2p_(nyawv)/(pi/8));
        dup=false; di=0;
        for j=1:nc
            jy=w2p_(nyaw_(j));
            if round(nx_(j))==kx && round(ny_(j))==ky && round(jy/(pi/8))==kyaw, dup=true; di=j; break; end
        end
        newg=ng(cur)+ds*(1+0.2*abs(d));
        h=hypot(gx-nxv,gy-nyv)+1.5*abs(adf(gyaw,nyawv)); newf=newg+1.4*h;
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
function c=iscol(x1,y1,x2,y2,m), c=false; for k=0:8, t=k/8;
    xx=x1+t*(x2-x1); yy=y1+t*(y2-y1);
    if xx<0||xx>100||yy<-100||yy>0, c=true; return; end
    r=floor(100-yy)+1; cc=floor(xx+100)+1;
    if r<1||r>200||cc<1||cc>200, c=true; return; end
    for rr=max(1,r-1):min(200,r+1)
        for ccol=max(1,cc-1):min(200,cc+1)
            if m(rr,ccol)>0.5, c=true; return; end
        end
    end
end
end
function a=wp_(a), while a>pi, a=a-2*pi; end; while a<-pi, a=a+2*pi; end; end
function a=w2p_(a), while a<0, a=a+2*pi; end; while a>=2*pi, a=a-2*pi; end; end
function d=adf(a,b), d=wp_(a-b); end
