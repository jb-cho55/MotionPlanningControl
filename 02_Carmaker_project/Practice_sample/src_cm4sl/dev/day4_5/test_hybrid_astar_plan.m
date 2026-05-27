function test_hybrid_astar_plan()
%TEST_HYBRID_ASTAR_PLAN  Standalone harness for hybrid_astar_plan.m.
%
%   Cases:
%     A. Free 100x100 grid, short straight (start->goal in front).
%     B. Free grid, 90-deg turn.
%     C. Free grid, parallel parking (reverse required).
%     D. Day4_5 Scenario 1 — 7 trucks + ego (0,-20) -> T00 (76.1,-5, pi/2).
%
%   For each case:
%     - path_len >= 2,
%     - last point within GOAL_TOL of goal,
%     - every consecutive segment is collision-free (re-check on occ grid).
%   PNG snapshots saved to dev/day4_5/figs/.

here = fileparts(mfilename('fullpath'));
addpath(here);
figs = fullfile(here, 'figs');
if ~exist(figs, 'dir'); mkdir(figs); end

c = map_const();
N = double(c.N);
x_min = c.X_MIN; x_max = c.X_MAX;
y_min = c.Y_MIN; y_max = c.Y_MAX;

GOAL_TOL = 1.7;  % planner uses 1.5; allow tiny margin for last-step jitter
veh_w = c.TRUCK_W; veh_l = c.TRUCK_L;
traffic_size = [veh_w, veh_l];

% Baseline: full parking lot rectangle => everywhere drivable.
boundary = [x_min y_min; x_max y_min; x_max y_max; x_min y_max];
base_free = generate_map(boundary);
empty_info = zeros(21, 1);
occ_free   = add_obstacle(base_free, empty_info, traffic_size);

results = struct('name', {}, 'plen', {}, 'goal_err', {}, 'ok', {});

%% --- Case A: short straight ---
fprintf('=== Case A: free grid, short straight ===\n');
[px, py, pyaw, plen] = hybrid_astar_plan(10, -50, 0, 25, -50, 0, occ_free);
results(end+1) = check_case('A_straight', px, py, pyaw, plen, 25, -50, 0, occ_free, GOAL_TOL, figs);

%% --- Case B: 90 deg turn ---
fprintf('\n=== Case B: free grid, 90-deg turn ===\n');
[px, py, pyaw, plen] = hybrid_astar_plan(10, -50, 0, 30, -30, pi/2, occ_free);
results(end+1) = check_case('B_turn', px, py, pyaw, plen, 30, -30, pi/2, occ_free, GOAL_TOL, figs);

%% --- Case C: parallel-parking-ish (reverse-friendly goal) ---
fprintf('\n=== Case C: free grid, goal in front but yaw=+pi/2 (reverse mix) ===\n');
[px, py, pyaw, plen] = hybrid_astar_plan(10, -50, 0, 20, -50, pi/2, occ_free);
results(end+1) = check_case('C_reverse_mix', px, py, pyaw, plen, 20, -50, pi/2, occ_free, GOAL_TOL, figs);

%% --- Case D: Day4_5 Scenario 1 full ---
fprintf('\n=== Case D: Day4_5 Scenario 1 ===\n');
trucks_deg = [40 -12  90;
              40 -24  90;
              40 -36  90;
              40 -48  90;
              39 -50   0;
              51 -50   0;
              63 -50   0];
info_d = zeros(21, 1);
for k = 1:7
    info_d((k-1)*3 + 1) = trucks_deg(k,1);
    info_d((k-1)*3 + 2) = trucks_deg(k,2);
    info_d((k-1)*3 + 3) = deg2rad(trucks_deg(k,3));
end
occ_d = add_obstacle(base_free, info_d, traffic_size);
% Goal option A: T00 rear bumper pose
gx_d = 76.1; gy_d = -5; gyaw_d = pi/2;
sx_d = 0;    sy_d = -20; syaw_d = 0;
fprintf('Planning %.1f -> %.1f (start (%g,%g,%g), goal (%g,%g,%g))\n', sx_d, gx_d, sx_d, sy_d, syaw_d, gx_d, gy_d, gyaw_d);
tic;
[px, py, pyaw, plen] = hybrid_astar_plan(sx_d, sy_d, syaw_d, gx_d, gy_d, gyaw_d, occ_d);
elapsed = toc;
fprintf('Plan time: %.2f s, path_len = %d\n', elapsed, plen);
results(end+1) = check_case('D_scenario1', px, py, pyaw, plen, gx_d, gy_d, gyaw_d, occ_d, GOAL_TOL, figs);

% Save the scenario-1 plot with all annotations
fig = figure('Visible','off','Position',[50 50 800 800]);
imagesc([x_min x_max], [y_max y_min], occ_d); colormap(flipud(gray)); axis xy equal tight; grid on;
hold on;
plot(sx_d, sy_d, 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(gx_d, gy_d, 'b*', 'MarkerSize', 14, 'LineWidth', 2);
if plen >= 2
    plot(px(1:plen), py(1:plen), 'r-', 'LineWidth', 1.8);
    quiver_step = max(1, floor(plen/12));
    for k = 1:quiver_step:plen
        u = cos(pyaw(k))*1.2; v = sin(pyaw(k))*1.2;
        quiver(px(k), py(k), u, v, 0, 'Color',[0.8 0.2 0.2], 'MaxHeadSize', 2);
    end
end
legend({'occ','ego start','T00 goal','path','heading'}, 'Location','southwest');
title(sprintf('Hybrid A* — Day4_5 Scenario 1 (plen=%d, %.2fs)', plen, elapsed));
xlabel('x [m]'); ylabel('y [m]');
saveas(fig, fullfile(figs, 'hybrid_astar_scenario1.png'));
close(fig);
fprintf('Saved figs/hybrid_astar_scenario1.png\n');

%% --- Summary ---
all_ok = true;
fprintf('\n--- Summary ---\n');
for i = 1:length(results)
    r = results(i);
    fprintf('  %-15s plen=%4d goal_err=%.2f  [%s]\n', r.name, r.plen, r.goal_err, ternary(r.ok, 'OK', 'FAIL'));
    if ~r.ok; all_ok = false; end
end
fprintf('\n--- RESULT: %s ---\n', ternary(all_ok, 'ALL PASS', 'FAIL'));
if ~all_ok
    error('test_hybrid_astar_plan: one or more cases failed.');
end

end

function r = check_case(name, px, py, pyaw, plen, gx, gy, gyaw, occ, tol, figs)
ok = false;
goal_err = inf;
if plen >= 2
    goal_err = hypot(px(plen) - gx, py(plen) - gy);
    yaw_err  = wrap_pi_abs(pyaw(plen) - gyaw);
    coll = false;
    for k = 1:plen-1
        if seg_hits(px(k), py(k), px(k+1), py(k+1), occ)
            coll = true; break;
        end
    end
    ok = (goal_err < tol) && (yaw_err < 0.5) && ~coll;
    fprintf('  plen=%d goal_err=%.2f yaw_err=%.2f coll=%d\n', plen, goal_err, yaw_err, coll);
else
    fprintf('  planning failed, plen=%d\n', plen);
end

% Visualisation
fig = figure('Visible','off','Position',[50 50 700 700]);
c = map_const();
imagesc([c.X_MIN c.X_MAX], [c.Y_MAX c.Y_MIN], occ); colormap(flipud(gray));
axis xy equal tight; grid on; hold on;
if plen >= 2
    plot(px(1:plen), py(1:plen), 'r-', 'LineWidth', 1.6);
end
plot(px(1), py(1), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(gx, gy, 'b*', 'MarkerSize', 14, 'LineWidth', 2);
title(sprintf('Hybrid A* case %s — plen=%d', name, plen));
xlabel('x [m]'); ylabel('y [m]');
saveas(fig, fullfile(figs, sprintf('hybrid_astar_%s.png', name)));
close(fig);

r = struct('name', name, 'plen', double(plen), 'goal_err', goal_err, 'ok', ok);
end

function hit = seg_hits(x1, y1, x2, y2, occ)
hit = false;
for k = 0:8
    t = k / 8;
    x = x1 + t*(x2 - x1);
    y = y1 + t*(y2 - y1);
    if pose_occupied(x, y, occ); hit = true; return; end
end
end

function occ = pose_occupied(x, y, m)
% Single-cell check (inflation already in occ map via add_obstacle).
c = map_const();
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

function a = wrap_pi_abs(a)
while a > pi;  a = a - 2*pi; end
while a < -pi; a = a + 2*pi; end
a = abs(a);
end

function out = ternary(cond, a, b)
if cond; out = a; else; out = b; end
end
