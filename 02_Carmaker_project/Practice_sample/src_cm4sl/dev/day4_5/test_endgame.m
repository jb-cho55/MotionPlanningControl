function test_endgame()
%TEST_ENDGAME  Closed-loop kinematic-bicycle sanity test for the Parking
%   chart, focused on the final parking yaw error.
%
%   Drives Parking() from the scenario start (0,-20) to the goal (76.1,-6,
%   pi/2) using a rear-axle kinematic bicycle model, and reports the final
%   distance + yaw error.  This is a CarMaker-free check so endgame tuning
%   can be validated before burning a CarMaker run (it will not match
%   CarMaker dynamics exactly, but validates the planner/controller logic).
%
%   PASS target: final |yaw_err| < 3 deg, final d_goal < 0.5 m, no collision.

here = fileparts(mfilename('fullpath'));
addpath(here);

% --- Map (scenario 1 trucks) ---
map_boundary = [0 0; 100 0; 100 -100; 0 -100];
trucks_deg = [40 -12 90; 40 -24 90; 40 -36 90; 40 -48 90; 39 -50 0; 51 -50 0; 63 -50 0];
ti = zeros(21, 1);
for k = 1:7
    ti((k-1)*3+1) = trucks_deg(k,1);
    ti((k-1)*3+2) = trucks_deg(k,2);
    ti((k-1)*3+3) = deg2rad(trucks_deg(k,3));
end
m1  = generate_map_(map_boundary, ti, [2.48 11.5]);
occ = add_obstacle_(m1, ti, [2.48 11.5]);

% --- Sim params ---
DT = 0.01; L = 2.8; T = 150; N = round(T/DT);
gx = 76.1; gy = -6; gyaw = pi/2;

x = 0; y = -20; yaw = 0; v = 0;
clear functions;     %#ok<CLFUNC>  reset all persistent state (Parking, pd_speed)

lx = zeros(1, N); ly = zeros(1, N); lyaw = zeros(1, N); lv = zeros(1, N);
done_k = N;
for k = 1:N
    [ax, sfl, ~, ~, ~, ~, sel] = Parking(x, y, yaw, v, [0 -20 0], [gx gy 0], gyaw, occ); %#ok<ASGLU>
    steer = sfl;

    % Kinematic bicycle (rear-axle reference, signed speed).
    v   = v + ax * DT;
    x   = x + v * cos(yaw) * DT;
    y   = y + v * sin(yaw) * DT;
    yaw = yaw + (v / L) * tan(steer) * DT;

    lx(k) = x; ly(k) = y; lyaw(k) = yaw; lv(k) = v;

    d = hypot(gx - x, gy - y);
    if d < 0.15 && abs(v) < 0.02 && k > 100
        done_k = k;
        break;
    end
end

lx = lx(1:done_k); ly = ly(1:done_k); lyaw = lyaw(1:done_k); lv = lv(1:done_k);

wrap = @(a) atan2(sin(a), cos(a));
yaw_err = rad2deg(wrap(lyaw(end) - gyaw));
d_end = hypot(gx - lx(end), gy - ly(end));

fprintf('=== test_endgame ===\n');
fprintf('  sim steps used : %d (%.1f s)\n', done_k, done_k*DT);
fprintf('  end pos        : (%.3f, %.3f)   goal (%.1f, %.1f)\n', lx(end), ly(end), gx, gy);
fprintf('  d_goal end     : %.3f m\n', d_end);
fprintf('  final yaw       : %.2f deg   (goal 90 deg)\n', rad2deg(lyaw(end)));
fprintf('  final yaw_err   : %.2f deg\n', yaw_err);

ok = abs(yaw_err) < 3.0 && d_end < 0.5;
fprintf('  RESULT: %s\n', tern(ok, 'PASS (yaw_err<3deg, d<0.5m)', 'FAIL'));

% Visual
fig = figure('Visible','off','Position',[60 60 900 900]);
imagesc([0 100], [0 -100], occ); colormap(flipud(gray)); axis xy equal tight; grid on; hold on;
plot(lx, ly, 'b-', 'LineWidth', 1.4);
plot(lx(1), ly(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(gx, gy, 'r*', 'MarkerSize', 14, 'LineWidth', 2);
% draw final ego heading arrow
quiver(lx(end), ly(end), 3*cos(lyaw(end)), 3*sin(lyaw(end)), 0, 'm', 'LineWidth', 2, 'MaxHeadSize', 2);
xlim([60 90]); ylim([-25 -2]);
title(sprintf('test\\_endgame: yaw\\_err=%.2f deg, d=%.2f m', yaw_err, d_end));
xlabel('x [m]'); ylabel('y [m]');
figs = fullfile(here, 'figs');
if ~exist(figs, 'dir'); mkdir(figs); end
saveas(fig, fullfile(figs, 'test_endgame.png'));
close(fig);
fprintf('  saved figs/test_endgame.png\n');
end

function s = tern(c, a, b)
if c; s = a; else; s = b; end
end
