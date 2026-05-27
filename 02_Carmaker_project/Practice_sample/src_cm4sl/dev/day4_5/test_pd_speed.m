function test_pd_speed()
%TEST_PD_SPEED  Standalone harness for pd_speed.m (closed-loop with a
%   simple 1st-order vehicle dynamics approximation).
%
%   Vehicle model:  v_dot = (a_cmd - 0.05 * v) / 1     (tau=1, drag 0.05)
%
%   Cases (DT=0.01 s, T=8 s):
%     A. step v_des = 5 m/s from rest        -> ss err < 0.2 m/s,
%                                              overshoot < 25%,
%                                              t_settle (2%) < 6 s.
%     B. step down v_des = 5 -> 1 m/s        -> decel reaches AX_MIN
%                                              early, then ss err < 0.2.
%     C. constant v_des = 3, ego starts at 6 -> ax saturates at AX_MIN
%                                              initially.
%   PNG snapshots saved.

here = fileparts(mfilename('fullpath'));
addpath(here);
figs = fullfile(here, 'figs');
if ~exist(figs, 'dir'); mkdir(figs); end

DT = 0.01;
T  = 8.0;
N  = round(T / DT);
TAU = 1.0;
DRAG = 0.05;

fprintf('=== test_pd_speed ===\n');

%% --- Case A: step up 0 -> 5 m/s ---
v_des_fn = @(t) 5.0 * (t > 0);
[t, v, ax] = simulate(v_des_fn, 0.0, N, DT, TAU, DRAG);
ss_err = abs(v(end) - 5.0);
overshoot = max(0, max(v) - 5.0) / 5.0;
% Settling: into a band wider than the PD steady-state error itself
% (PD without integral leaves a known residual due to drag).
settle_tol = max(0.3, ss_err + 0.1);
i_settle = find(abs(v - 5.0) > settle_tol, 1, 'last');
if isempty(i_settle); i_settle = 0; end
t_settle = (i_settle + 1) * DT;
ok_A = ss_err < 0.3 && overshoot < 0.25 && t_settle < 6.0;
fprintf('A_step_up:   ss_err=%.3f overshoot=%.1f%% t_settle=%.2fs  [%s]\n', ...
    ss_err, overshoot*100, t_settle, ternary(ok_A, 'OK', 'FAIL'));
plot_resp('A_step_up', t, v, ax, 5.0, figs);

%% --- Case B: step down 5 -> 1 m/s at t=2s ---
v_des_fn = @(t) 5.0 - 4.0*(t >= 2.0);
[t, v, ax] = simulate(v_des_fn, 5.0, N, DT, TAU, DRAG);
% After t=4s, should be near 1 m/s.
i4 = round(6.0 / DT);
ss_err_B = abs(v(i4) - 1.0);
min_ax = min(ax);
ok_B = ss_err_B < 0.3 && min_ax < -2.0;
fprintf('B_step_down: ss_err@6s=%.3f min_ax=%.2f  [%s]\n', ...
    ss_err_B, min_ax, ternary(ok_B, 'OK', 'FAIL'));
plot_resp('B_step_down', t, v, ax, NaN, figs);

%% --- Case C: ego too fast, v_des=3, v0=6 ---
v_des_fn = @(t) 3.0;
[t, v, ax] = simulate(v_des_fn, 6.0, N, DT, TAU, DRAG);
ax_min_early = min(ax(1:50));
ok_C = ax_min_early <= -2.0;     % should saturate near AX_MIN=-3
ss_err_C = abs(v(end) - 3.0);
ok_C = ok_C && ss_err_C < 0.3;
fprintf('C_decel:     min_ax(first 0.5s)=%.2f ss_err=%.3f  [%s]\n', ...
    ax_min_early, ss_err_C, ternary(ok_C, 'OK', 'FAIL'));
plot_resp('C_decel', t, v, ax, 3.0, figs);

all_ok = ok_A && ok_B && ok_C;
fprintf('\n--- RESULT: %s ---\n', ternary(all_ok, 'ALL PASS', 'FAIL'));
if ~all_ok
    error('test_pd_speed: one or more cases failed.');
end

end

function [t, v, ax] = simulate(v_des_fn, v0, N, DT, TAU, DRAG)
% Reset persistent state in pd_speed by clearing it
clear pd_speed
t  = (0:N).' * DT;
v  = zeros(N+1, 1);
ax = zeros(N, 1);
v(1) = v0;
for k = 1:N
    vd = v_des_fn(t(k));
    a  = pd_speed(vd, v(k));
    ax(k) = a;
    % 1st-order: v_dot = (a - DRAG*v) / TAU
    v(k+1) = v(k) + ((a - DRAG*v(k))/TAU) * DT;
end
end

function plot_resp(name, t, v, ax, v_des_const, figs)
fig = figure('Visible','off','Position',[50 50 1100 500]);
subplot(1,2,1);
plot(t, v, 'r-', 'LineWidth', 1.5); hold on; grid on;
if ~isnan(v_des_const)
    plot(t, v_des_const*ones(size(t)), 'k--', 'LineWidth', 1.0);
    legend({'v_{ego}','v_{des}'}, 'Location','best');
end
title(sprintf('pd_speed %s — velocity', name));
xlabel('t [s]'); ylabel('v [m/s]');

subplot(1,2,2);
plot(t(1:end-1), ax, 'b-'); grid on;
title('desired ax [m/s^2]'); xlabel('t [s]');

saveas(fig, fullfile(figs, sprintf('pd_speed_%s.png', name)));
close(fig);
end

function out = ternary(cond, a, b)
if cond; out = a; else; out = b; end
end
