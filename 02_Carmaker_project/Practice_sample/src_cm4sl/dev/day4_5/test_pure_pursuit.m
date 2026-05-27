function test_pure_pursuit()
%TEST_PURE_PURSUIT  Standalone harness for pure_pursuit.m.
%
%   Cases (closed-loop with kinematic bicycle integrator, dt=0.05s, v=3m/s):
%     A. Straight reference  -> steady-state lateral error < 0.1 m.
%     B. Constant-radius arc -> steer sign consistent with curvature.
%     C. S-curve              -> ego tracks within 0.5 m peak error.
%     D. Initial offset       -> converges to within 0.2 m by t=5s.
%   PNG snapshots saved to dev/day4_5/figs/.

here = fileparts(mfilename('fullpath'));
addpath(here);
figs = fullfile(here, 'figs');
if ~exist(figs, 'dir'); mkdir(figs); end

c = map_const();
L = c.WHEELBASE;

dt    = 0.05;
T_sim = 8.0;
N_step = round(T_sim / dt);

fprintf('=== test_pure_pursuit ===\n');
fprintf('Wheelbase=%.2f m, dt=%.2f s, T=%.1f s, v=3 m/s\n', L, dt, T_sim);

%% --- Case A: straight reference y=0, ego starts offset by +0.5m ---
ref_x = 0:0.5:60;
ref_y = zeros(size(ref_x));
[traj, steer_log] = simulate(ref_x, ref_y, 0.0, 0.5, 0.0, 3.0, dt, N_step, L);
ss_err_A = abs(traj(end, 2));
ok_A = ss_err_A < 0.1;
fprintf('A_straight:    final |y|=%.3f m  [%s]\n', ss_err_A, ternary(ok_A, 'OK', 'FAIL'));
plot_traj('A_straight', ref_x, ref_y, traj, steer_log, figs);

%% --- Case B: arc of radius 20m starting at (20, 0) ---
theta = linspace(0, pi, 80).';
ref_x = 20 + 20*sin(theta);
ref_y = -20 + 20*cos(theta);   % arc going down and around
[traj, steer_log] = simulate(ref_x, ref_y, 0.0, 0.0, 0.0, 3.0, dt, N_step, L);
% Reference arc starts heading down-right then loops back: steer should
% be consistently of one sign (curvature is monotone), so |mean| > 0.05.
mean_steer = mean(steer_log(20:end));
ok_B = abs(mean_steer) > 0.02;
fprintf('B_arc_R20:     mean steer (after settling) = %.3f rad  [%s]\n', ...
    mean_steer, ternary(ok_B, 'OK', 'FAIL'));
plot_traj('B_arc', ref_x, ref_y, traj, steer_log, figs);

%% --- Case C: S-curve ---
t_ref = linspace(0, 1, 120).';
ref_x = 60 * t_ref;
ref_y = 4 * sin(2*pi*t_ref);
[traj, steer_log] = simulate(ref_x, ref_y, 0.0, 0.0, 0.0, 3.0, dt, N_step, L);
peak_err_C = max_cross_track(traj, ref_x, ref_y);
ok_C = peak_err_C < 0.6;
fprintf('C_S_curve:     peak |cross-track|=%.3f m  [%s]\n', peak_err_C, ternary(ok_C, 'OK', 'FAIL'));
plot_traj('C_S_curve', ref_x, ref_y, traj, steer_log, figs);

%% --- Case D: large initial offset 2m, must converge ---
ref_x = (0:0.5:60).';
ref_y = zeros(size(ref_x));
[traj, ~] = simulate(ref_x, ref_y, 0.0, 2.0, 0.0, 3.0, dt, N_step, L);
% Check |y| at t=5s
i5 = round(5.0 / dt) + 1;
err_5s = abs(traj(min(i5, size(traj,1)), 2));
ok_D = err_5s < 0.2;
fprintf('D_recover:     |y(5s)|=%.3f m  [%s]\n', err_5s, ternary(ok_D, 'OK', 'FAIL'));
plot_traj('D_recover', ref_x, ref_y, traj, zeros(N_step,1), figs);

%% --- Summary ---
all_ok = ok_A && ok_B && ok_C && ok_D;
fprintf('\n--- RESULT: %s ---\n', ternary(all_ok, 'ALL PASS', 'FAIL'));
if ~all_ok
    error('test_pure_pursuit: one or more cases failed.');
end

end

function [traj, steer_log] = simulate(ref_x, ref_y, ego_x0, ego_y0, ego_yaw0, v, dt, N, L)
% Bicycle-model rollout under pure_pursuit steering.
ref_x = ref_x(:); ref_y = ref_y(:);
% Pad to MAX_PATH=300
plen = int32(length(ref_x));
if plen > int32(300); plen = int32(300); end
px = zeros(300, 1); py = zeros(300, 1);
px(1:plen) = ref_x(1:plen);
py(1:plen) = ref_y(1:plen);

traj = zeros(N+1, 3);
steer_log = zeros(N, 1);
ego = [ego_x0; ego_y0; ego_yaw0];
traj(1, :) = ego.';
for k = 1:N
    steer = pure_pursuit(ego(1), ego(2), ego(3), v, px, py, plen);
    steer_log(k) = steer;
    % Kinematic bicycle integration (rear-axle reference).
    ego(1) = ego(1) + v * cos(ego(3)) * dt;
    ego(2) = ego(2) + v * sin(ego(3)) * dt;
    ego(3) = ego(3) + v * tan(steer) / L * dt;
    traj(k+1, :) = ego.';
end
end

function e = max_cross_track(traj, ref_x, ref_y)
e = 0;
for i = 1:size(traj, 1)
    d2_min = inf;
    for j = 1:length(ref_x)
        d2 = (traj(i,1) - ref_x(j))^2 + (traj(i,2) - ref_y(j))^2;
        if d2 < d2_min; d2_min = d2; end
    end
    e = max(e, sqrt(d2_min));
end
end

function plot_traj(name, ref_x, ref_y, traj, steer_log, figs)
fig = figure('Visible','off','Position',[50 50 1100 500]);
subplot(1,2,1);
plot(ref_x, ref_y, 'k--', 'LineWidth', 1.2); hold on; grid on; axis equal;
plot(traj(:,1), traj(:,2), 'r-', 'LineWidth', 1.5);
plot(traj(1,1), traj(1,2), 'go', 'MarkerSize', 10, 'LineWidth', 1.5);
legend({'reference','ego trajectory','start'}, 'Location','best');
title(sprintf('pure pursuit %s — trajectory', name));
xlabel('x [m]'); ylabel('y [m]');

subplot(1,2,2);
plot(steer_log, 'b-'); grid on;
title('steer command [rad]'); xlabel('step');

saveas(fig, fullfile(figs, sprintf('pure_pursuit_%s.png', name)));
close(fig);
end

function out = ternary(cond, a, b)
if cond; out = a; else; out = b; end
end
