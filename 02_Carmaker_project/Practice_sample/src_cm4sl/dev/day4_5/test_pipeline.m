function test_pipeline()
%TEST_PIPELINE  End-to-end closed-loop with all 4 dev modules tied
%   together by planner_top, ego driven by a kinematic bicycle.
%
%   Scenario: Day4_5 Scenario 1 (T01..T07 trucks, T00 goal at
%             (76.1, -5, pi/2)).  Ego starts at (0, -20, 0, 0 m/s).
%
%   Pass criteria:
%     - Sim runs without numerical error.
%     - Ego trajectory never enters an occupied cell (collision = 0).
%     - Distance to T00 drops below 3 m by end of sim.
%   PNG: ego trajectory over the obstacle grid + speed and steer time
%        series saved to dev/day4_5/figs/.

here = fileparts(mfilename('fullpath'));
addpath(here);
figs = fullfile(here, 'figs');
if ~exist(figs, 'dir'); mkdir(figs); end

c = map_const();

% --- Scenario inputs ---
% Ego starts slightly inside the parking lot so the road-edge inflation
% (ego_w/2 + safety = 1.75 m) does not flag the spawn cell as occupied.
% CarMaker TestRun still spawns at (0, -20); whether to move that too is
% a separate scenario decision.
ego = [2; -20; 0; 0];                 % x, y, yaw, v
% Goal box rear bumper (now shifted 1 m inside the parking lot so the
% full 6 m box stays clear of the y=0 road edge).
t00 = [76.1; -6; pi/2];
trucks_deg = [40 -12  90;
              40 -24  90;
              40 -36  90;
              40 -48  90;
              39 -50   0;
              51 -50   0;
              63 -50   0];
traffic_info = zeros(21, 1);
for k = 1:7
    traffic_info((k-1)*3 + 1) = trucks_deg(k, 1);
    traffic_info((k-1)*3 + 2) = trucks_deg(k, 2);
    traffic_info((k-1)*3 + 3) = deg2rad(trucks_deg(k, 3));
end
traffic_size = [c.TRUCK_W, c.TRUCK_L];
map_boundary = [c.X_MIN c.Y_MIN;
                c.X_MAX c.Y_MIN;
                c.X_MAX c.Y_MAX;
                c.X_MIN c.Y_MAX];

% --- Sim params ---
DT = 0.01;     % 100 Hz in this .m harness (CarMaker .slx uses 1 kHz)
T  = 90.0;     % long enough for the L-detour + endgame approach
N  = round(T / DT);
TAU  = 1.0;    % 1st-order vehicle lag
DRAG = 0.05;

% --- Reset persistent state (planner_top, pd_speed) ---
clear planner_top
clear pd_speed

% --- Logs ---
ego_log   = zeros(N+1, 4);
ax_log    = zeros(N, 1);
steer_log = zeros(N, 1);
ego_log(1, :) = ego.';
collision = false;
path_snap_x = []; path_snap_y = []; occ_snap = [];

fprintf('=== test_pipeline ===\n');
fprintf('DT=%.2fs, T=%.1fs, N=%d steps\n', DT, T, N);
fprintf('Ego start: (%.1f, %.1f, %.2f), goal T00: (%.1f, %.1f, %.2f)\n', ...
    ego(1), ego(2), ego(3), t00(1), t00(2), t00(3));

t0 = tic;
for k = 1:N
    [steer_fl, ~, ax_cmd, path_x_dbg, path_y_dbg, plen_dbg, occ_dbg] = ...
        planner_top(ego(1), ego(2), ego(3), ego(4), ...
                    t00(1), t00(2), t00(3), ...
                    traffic_info, traffic_size, map_boundary);
    steer_log(k) = steer_fl;
    ax_log(k)    = ax_cmd;

    % Snapshot at t=1s (after first plan) for visualisation
    if k == round(1.0/DT)
        path_snap_x = path_x_dbg;
        path_snap_y = path_y_dbg;
        occ_snap    = occ_dbg;
        snap_plen   = plen_dbg;
    end

    % Bicycle + 1st-order longitudinal lag
    L = c.WHEELBASE;
    v = ego(4);
    ego(1) = ego(1) + v * cos(ego(3)) * DT;
    ego(2) = ego(2) + v * sin(ego(3)) * DT;
    ego(3) = ego(3) + v * tan(steer_fl) / L * DT;
    ego(4) = ego(4) + ((ax_cmd - DRAG*v)/TAU) * DT;
    if ego(4) < 0; ego(4) = 0; end          % clamp non-negative (no reverse in this test)
    ego_log(k+1, :) = ego.';

    % Collision check against the snapshot occ
    if ~isempty(occ_snap)
        if pose_occupied(ego(1), ego(2), occ_snap, c)
            if ~collision
                fprintf('!! collision at step %d, t=%.2fs, ego=(%.2f, %.2f)\n', ...
                    k, k*DT, ego(1), ego(2));
            end
            collision = true;
        end
    end
end
elapsed = toc(t0);
fprintf('Simulated %.1f s in %.2f s wall time.\n', T, elapsed);

% --- Metrics ---
d_end = hypot(ego(1) - t00(1), ego(2) - t00(2));
fprintf('Final ego: (%.2f, %.2f, %.2f), v=%.2f m/s\n', ego(1), ego(2), ego(3), ego(4));
fprintf('Distance to T00: %.2f m\n', d_end);
fprintf('Collision flag: %d\n', collision);
reached = d_end < 3.0;

% --- Trajectory plot ---
fig = figure('Visible','off','Position',[50 50 900 900]);
if ~isempty(occ_snap)
    imagesc([c.X_MIN c.X_MAX], [c.Y_MAX c.Y_MIN], occ_snap);
    colormap(flipud(gray)); axis xy equal tight; grid on; hold on;
else
    axis equal; grid on; hold on;
end
if ~isempty(path_snap_x) && snap_plen >= 2
    plot(path_snap_x(1:snap_plen), path_snap_y(1:snap_plen), 'r--', 'LineWidth', 1.2);
end
plot(ego_log(:,1), ego_log(:,2), 'b-', 'LineWidth', 1.6);
plot(ego_log(1,1), ego_log(1,2), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(t00(1), t00(2), 'r*', 'MarkerSize', 14, 'LineWidth', 2);
xlim([c.X_MIN c.X_MAX]); ylim([c.Y_MIN c.Y_MAX]);
title(sprintf('test_pipeline — ego trajectory (d_{end}=%.2f m, coll=%d)', d_end, collision));
xlabel('x [m]'); ylabel('y [m]');
legend({'occ','planned path (t=1s)','ego trajectory','start','T00 goal'}, 'Location','southwest');
saveas(fig, fullfile(figs, 'pipeline_traj.png'));
close(fig);

% --- Time series plot ---
fig2 = figure('Visible','off','Position',[50 50 1200 700]);
t = (0:N).' * DT;
subplot(3,1,1);
plot(t, ego_log(:,4), 'r-', 'LineWidth', 1.2); grid on;
title('Ego v [m/s]'); xlabel('t [s]');
subplot(3,1,2);
plot(t(1:end-1), ax_log, 'b-'); grid on;
title('Desired ax [m/s^2]'); xlabel('t [s]');
subplot(3,1,3);
plot(t(1:end-1), steer_log, 'k-'); grid on;
title('Steer [rad]'); xlabel('t [s]');
saveas(fig2, fullfile(figs, 'pipeline_signals.png'));
close(fig2);

fprintf('Saved figs/pipeline_traj.png, figs/pipeline_signals.png\n');

ok = ~collision && reached;
fprintf('\n--- RESULT: %s ---\n', ternary(ok, 'PASS', 'FAIL'));
if ~ok
    error('test_pipeline: collision=%d  reached=%d  d_end=%.2f m', collision, reached, d_end);
end

end

function occ = pose_occupied(x, y, m, c)
res = c.RES;
N = double(c.N);
occ = false;
if x < c.X_MIN || x > c.X_MAX || y < c.Y_MIN || y > c.Y_MAX
    occ = true; return;
end
col = floor((x - c.X_MIN)/res) + 1;
row = floor((c.Y_MAX - y)/res) + 1;
if row < 1 || row > N || col < 1 || col > N; occ = true; return; end
if m(row, col) > 0; occ = true; end
end

function out = ternary(cond, a, b)
if cond; out = a; else; out = b; end
end
