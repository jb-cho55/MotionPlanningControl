function test_stanley()
%TEST_STANLEY  Standalone harness for stanley.m.
%
%   Cases:
%     A. Straight reference + 0.5 m offset -> ss |y| < 0.1 m.
%     B. S-curve                          -> peak |cross-track| < 0.6 m.
%     C. Goal-yaw alignment near path end -> final |yaw - goal_yaw| < 0.15 rad.
%     D. Heading error only (no offset)   -> ego yaw converges to path yaw.
%   PNG snapshots saved.

here = fileparts(mfilename('fullpath'));
addpath(here);
figs = fullfile(here, 'figs');
if ~exist(figs, 'dir'); mkdir(figs); end

c = map_const();
L = c.WHEELBASE;
dt = 0.05;
T = 12.0;
N = round(T/dt);

fprintf('=== test_stanley ===\n');

%% --- A: straight reference + offset ---
ref_x = (0:0.5:60).';
ref_y = zeros(size(ref_x));
ref_yaw = zeros(size(ref_x));
[traj, ~] = simulate(ref_x, ref_y, ref_yaw, 0.0, 0.5, 0.0, 3.0, dt, N, L, 0.0);
ss_err_A = abs(traj(end, 2));
ok_A = ss_err_A < 0.1;
fprintf('A_straight: ss |y|=%.3f m  [%s]\n', ss_err_A, ternary(ok_A, 'OK', 'FAIL'));
plot_traj('A_straight', ref_x, ref_y, traj, figs);

%% --- B: S-curve ---
t_ref = linspace(0, 1, 200).';
ref_x = 60 * t_ref;
ref_y = 4 * sin(2*pi*t_ref);
ref_yaw = zeros(size(ref_x));
for i = 2:length(ref_x)
    ref_yaw(i) = atan2(ref_y(i)-ref_y(i-1), ref_x(i)-ref_x(i-1));
end
ref_yaw(1) = ref_yaw(2);
[traj, ~] = simulate(ref_x, ref_y, ref_yaw, 0.0, 0.0, 0.0, 3.0, dt, N, L, 0.0);
peak_C = max_cross_track(traj, ref_x, ref_y);
ok_B = peak_C < 0.6;
fprintf('B_S_curve:  peak |cross-track|=%.3f m  [%s]\n', peak_C, ternary(ok_B, 'OK', 'FAIL'));
plot_traj('B_S_curve', ref_x, ref_y, traj, figs);

%% --- C: goal-yaw alignment near end ---
% Short reference (10 m east) so ego reaches the END_RADIUS region within
% the sim window; goal_yaw asks for +pi/2 alignment at termination.
ref_x = (0:0.5:10).';
ref_y = zeros(size(ref_x));
ref_yaw = zeros(size(ref_x));
goal_yaw = pi/2;
[traj, yaw_log] = simulate(ref_x, ref_y, ref_yaw, 0.0, 0.0, 0.0, 2.0, dt, N, L, goal_yaw);
% Check yaw progress: after entering END_RADIUS (last few seconds), ego
% should be at least halfway from 0 to goal_yaw.
yaw_err_end = abs(wrap_pi_local(yaw_log(end) - goal_yaw));
ok_C = yaw_err_end < 0.5;
fprintf('C_goal_yaw: final |yaw - goal_yaw|=%.3f rad (%.1f deg)  [%s]\n', ...
    yaw_err_end, rad2deg(yaw_err_end), ternary(ok_C, 'OK', 'FAIL'));
plot_traj_with_goal('C_goal_yaw', ref_x, ref_y, traj, yaw_log, goal_yaw, figs);

%% --- D: heading error only (ego starts yaw=+0.5 on a flat reference) ---
ref_x = (0:0.5:60).';
ref_y = zeros(size(ref_x));
ref_yaw = zeros(size(ref_x));
[traj, yaw_log] = simulate(ref_x, ref_y, ref_yaw, 0.0, 0.0, 0.5, 3.0, dt, N, L, 0.0);
ss_yaw_D = abs(yaw_log(end));
ok_D = ss_yaw_D < 0.05;
fprintf('D_yaw_only: ss |ego_yaw|=%.3f rad  [%s]\n', ss_yaw_D, ternary(ok_D, 'OK', 'FAIL'));
plot_traj('D_yaw_only', ref_x, ref_y, traj, figs);

all_ok = ok_A && ok_B && ok_C && ok_D;
fprintf('\n--- RESULT: %s ---\n', ternary(all_ok, 'ALL PASS', 'FAIL'));
if ~all_ok
    error('test_stanley: one or more cases failed.');
end

end


function [traj, yaw_log] = simulate(ref_x, ref_y, ref_yaw, ego_x0, ego_y0, ego_yaw0, v, dt, N, L, goal_yaw)
ref_x = ref_x(:); ref_y = ref_y(:); ref_yaw = ref_yaw(:);
plen = int32(length(ref_x));
if plen > int32(300); plen = int32(300); end
px = zeros(300, 1); py = zeros(300, 1); pyaw = zeros(300, 1);
px(1:plen)   = ref_x(1:plen);
py(1:plen)   = ref_y(1:plen);
pyaw(1:plen) = ref_yaw(1:plen);

traj    = zeros(N+1, 3);
yaw_log = zeros(N+1, 1);
ego = [ego_x0; ego_y0; ego_yaw0];
traj(1, :)  = ego.';
yaw_log(1)  = ego(3);
for k = 1:N
    % All-forward direction for these synthetic reference paths.
    pdir = ones(300, 1, 'int8');
    steer = stanley(ego(1), ego(2), ego(3), v, px, py, pyaw, pdir, plen, goal_yaw);
    ego(1) = ego(1) + v * cos(ego(3)) * dt;
    ego(2) = ego(2) + v * sin(ego(3)) * dt;
    ego(3) = ego(3) + v * tan(steer) / L * dt;
    traj(k+1, :)  = ego.';
    yaw_log(k+1) = ego(3);
end
end

function e = max_cross_track(traj, ref_x, ref_y)
e = 0;
for i = 1:size(traj, 1)
    d2 = inf;
    for j = 1:length(ref_x)
        dd = (traj(i,1) - ref_x(j))^2 + (traj(i,2) - ref_y(j))^2;
        if dd < d2; d2 = dd; end
    end
    e = max(e, sqrt(d2));
end
end

function plot_traj(name, ref_x, ref_y, traj, figs)
fig = figure('Visible','off','Position',[50 50 700 500]);
plot(ref_x, ref_y, 'k--', 'LineWidth', 1.2); hold on; grid on; axis equal;
plot(traj(:,1), traj(:,2), 'r-', 'LineWidth', 1.5);
plot(traj(1,1), traj(1,2), 'go', 'MarkerSize', 10, 'LineWidth', 1.5);
legend({'reference','ego','start'}, 'Location','best');
title(sprintf('stanley %s', name));
xlabel('x [m]'); ylabel('y [m]');
saveas(fig, fullfile(figs, sprintf('stanley_%s.png', name)));
close(fig);
end

function plot_traj_with_goal(name, ref_x, ref_y, traj, yaw_log, goal_yaw, figs)
fig = figure('Visible','off','Position',[50 50 1100 500]);
subplot(1,2,1);
plot(ref_x, ref_y, 'k--', 'LineWidth', 1.2); hold on; grid on; axis equal;
plot(traj(:,1), traj(:,2), 'r-', 'LineWidth', 1.5);
plot(traj(1,1), traj(1,2), 'go', 'MarkerSize', 10);
% Arrow at end showing actual heading vs goal
qx = traj(end,1); qy = traj(end,2);
quiver(qx, qy, 2*cos(traj(end,3)), 2*sin(traj(end,3)), 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 4);
quiver(qx, qy, 2*cos(goal_yaw), 2*sin(goal_yaw), 0, 'b--', 'LineWidth', 1.5, 'MaxHeadSize', 4);
legend({'ref','ego','start','ego yaw','goal yaw'}, 'Location','best');
title(sprintf('stanley %s — trajectory + final heading', name));
xlabel('x [m]'); ylabel('y [m]');

subplot(1,2,2);
plot(yaw_log, 'b-'); hold on; grid on;
yline(goal_yaw, 'r--', 'goal yaw');
title('ego yaw [rad]'); xlabel('step');
saveas(fig, fullfile(figs, sprintf('stanley_%s.png', name)));
close(fig);
end

function a = wrap_pi_local(a)
while a > pi;  a = a - 2*pi; end
while a < -pi; a = a + 2*pi; end
end

function out = ternary(cond, a, b)
if cond; out = a; else; out = b; end
end
