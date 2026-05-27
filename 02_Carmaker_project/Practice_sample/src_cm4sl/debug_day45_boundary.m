function debug_day45_boundary()
traffic_all_deg = [76.1 -5 90; 40 -12 90; 40 -24 90; 40 -36 90; 40 -48 90; 39 -50 0; 51 -50 0; 63 -50 0];
ti = zeros(21,1);
for k=1:7
    ti((k-1)*3+1)=traffic_all_deg(k,1); ti((k-1)*3+2)=traffic_all_deg(k,2); ti((k-1)*3+3)=deg2rad(traffic_all_deg(k,3));
end
y = zeros(200,200); y(1,:)=1; y(end,:)=1; y(:,1)=1; y(:,end)=1;
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
occ = y;

[px_a, py_a, ~, plen_a] = run_plan(0,-20,0, 80,-3,0, occ, -2);
if plen_a > 0
    fprintf('road_y_max=-2:  plen=%d, ends at (%.2f,%.2f) y_min=%.2f y_max=%.2f\n', plen_a, px_a(plen_a), py_a(plen_a), min(py_a(1:plen_a)), max(py_a(1:plen_a)));
else
    fprintf('road_y_max=-2:  NO PATH FOUND\n');
end

[px_b, py_b, ~, plen_b] = run_plan(0,-20,0, 80,-3,0, occ, 0);
if plen_b > 0
    fprintf('road_y_max= 0:  plen=%d, ends at (%.2f,%.2f) y_min=%.2f y_max=%.2f\n', plen_b, px_b(plen_b), py_b(plen_b), min(py_b(1:plen_b)), max(py_b(1:plen_b)));
else
    fprintf('road_y_max= 0:  NO PATH FOUND\n');
end

figure('Position',[100 100 1100 500]);
hold on; grid on; axis equal;
imagesc([-100 100], [100 -100], occ); colormap(flipud(gray)); axis xy;
xlim([-5 105]); ylim([-25 5]);
if plen_a>0, plot(px_a(1:plen_a), py_a(1:plen_a), 'b.-', 'LineWidth',1.5,'DisplayName','old (y_{max}=-2)'); end
if plen_b>0, plot(px_b(1:plen_b), py_b(1:plen_b), 'm.-', 'LineWidth',1.5,'DisplayName','fixed (y_{max}=0)'); end
plot(0,-20,'go','MarkerSize',10,'LineWidth',2);
plot(80,-3,'r*','MarkerSize',14);
plot([0 100],[-2 -2],'b--','LineWidth',1,'DisplayName','old boundary y=-2');
plot([0 100],[0 0],'m--','LineWidth',1,'DisplayName','fixed boundary y=0');
legend('Location','south');
title('Effect of road_{y,max} on path planning');
saveas(gcf, fullfile(fileparts(mfilename('fullpath')), 'debug_day45_boundary.png'));
end

function [px, py, pyaw, plen] = run_plan(sx, sy, syaw, gx, gy, gyaw, occ, ymax)
fprintf('  run_plan: start=(%.1f,%.1f,%.2f) goal=(%.1f,%.1f,%.2f) ymax=%.1f\n', sx, sy, syaw, gx, gy, gyaw, ymax);
MAX_NODES=4500; MAX_PATH=220;
px=zeros(MAX_PATH,1); py=zeros(MAX_PATH,1); pyaw=zeros(MAX_PATH,1); plen=0;
nx_=zeros(MAX_NODES,1); ny_=zeros(MAX_NODES,1); nyaw_=zeros(MAX_NODES,1);
ng=zeros(MAX_NODES,1); nf=zeros(MAX_NODES,1); npar=zeros(MAX_NODES,1);
nopen=false(MAX_NODES,1); nclosed=false(MAX_NODES,1);
steers=[-0.5,-0.32,-0.16,0,0.16,0.32,0.5]; L=2.8; ds=1.2;
nc=1; nx_(1)=sx; ny_(1)=sy; nyaw_(1)=syaw; nf(1)=hypot(gx-sx,gy-sy); nopen(1)=true;
gid=0;
for it=1:MAX_NODES
    cur=0; bf=1e12;
    for i=1:MAX_NODES, if nopen(i)&&nf(i)<bf, bf=nf(i); cur=i; end; end
    if cur==0
        fprintf('    iter=%d: no open nodes, nc=%d\n', it, nc);
        break;
    end
    nopen(cur)=false; nclosed(cur)=true;
    cx=nx_(cur); cy=ny_(cur); cyaw=nyaw_(cur);
    if it == 1 || mod(it, 50) == 0
        fprintf('    iter=%d cur=%d at (%.1f,%.1f,%.2f) f=%.1f, nc=%d\n', it, cur, cx, cy, cyaw, bf, nc);
    end
    if hypot(gx-cx,gy-cy)<2 && abs(cyaw-gyaw)<0.8, gid=cur; break; end
    for a=1:7
        d=steers(a);
        if abs(d)<1e-6, nxv=cx+ds*cos(cyaw); nyv=cy+ds*sin(cyaw); nyawv=cyaw;
        else, beta=ds*tan(d)/L; r=L/tan(d); nxv=cx+r*(sin(cyaw+beta)-sin(cyaw)); nyv=cy-r*(cos(cyaw+beta)-cos(cyaw)); nyawv=cyaw+beta; end
        while nyawv>pi, nyawv=nyawv-2*pi; end; while nyawv<-pi, nyawv=nyawv+2*pi; end
        coll=false;
        for kk=0:8
            t=kk/8; xx=cx+t*(nxv-cx); yy=cy+t*(nyv-cy);
            if xx<0||xx>100||yy<-100||yy>ymax, coll=true; break; end
            r1=floor(100-yy)+1; c1=floor(xx+100)+1;
            if r1<1||r1>200||c1<1||c1>200, coll=true; break; end
            for rr=max(1,r1-1):min(200,r1+1), if coll, break; end
                for cc=max(1,c1-1):min(200,c1+1), if occ(rr,cc)>0.5, coll=true; break; end; end
            end
            if coll, break; end
        end
        if coll, continue; end
        kx=round(nxv); ky=round(nyv);
        y2pi=nyawv; while y2pi<0, y2pi=y2pi+2*pi; end; while y2pi>=2*pi, y2pi=y2pi-2*pi; end
        kyaw=round(y2pi/(pi/8));
        dup=false; di=0;
        for j=1:nc
            jy=nyaw_(j); while jy<0, jy=jy+2*pi; end; while jy>=2*pi, jy=jy-2*pi; end
            if round(nx_(j))==kx && round(ny_(j))==ky && round(jy/(pi/8))==kyaw, dup=true; di=j; break; end
        end
        ayd=nyawv-gyaw; while ayd>pi, ayd=ayd-2*pi; end; while ayd<-pi, ayd=ayd+2*pi; end
        newg=ng(cur)+ds*(1+0.2*abs(d));
        h=hypot(gx-nxv,gy-nyv)+1.5*abs(ayd); newf=newg+1.4*h;
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
