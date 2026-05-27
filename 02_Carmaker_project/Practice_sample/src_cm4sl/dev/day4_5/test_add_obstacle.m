function test_add_obstacle()
%TEST_ADD_OBSTACLE  Standalone harness for generate_map.m + add_obstacle.m.
%
%   Two stages are exercised:
%     (A) generate_map(map_boundary) -> 200x200 baseline (off-road = 1).
%     (B) add_obstacle(base, traffic_info, traffic_size) -> baseline OR trucks.
%
%   Verified properties:
%     1. Empty / dummy traffic on a free baseline -> grid unchanged.
%     2. Single truck @ known pose: extra-marked cell count ~= footprint area.
%     3. Day4_5 Scenario 1 trucks produce a plausible occupancy.
%     4. T00 parking goal (76.1, -5) stays free in the trucks-only overlay.
%     5. PNG visualisations saved to dev/day4_5/figs/.

here = fileparts(mfilename('fullpath'));
addpath(here);
figs = fullfile(here, 'figs');
if ~exist(figs, 'dir'); mkdir(figs); end

c = map_const();
N = double(c.N);
res = c.RES;
x_min = c.X_MIN; x_max = c.X_MAX;
y_min = c.Y_MIN; y_max = c.Y_MAX;
margin = c.EGO_W*0.5 + c.SAFETY_MARGIN;
veh_w = c.TRUCK_W;
veh_l = c.TRUCK_L;
traffic_size = [veh_w, veh_l];

expected_area_m2 = (veh_l + 2*margin) * (veh_w + 2*margin);
expected_cells = expected_area_m2 / (res*res);

fprintf('=== test generate_map + add_obstacle ===\n');
fprintf('Grid: %dx%d, res=%.2f m, x=[%.0f,%.0f], y=[%.0f,%.0f]\n', ...
    N, N, res, x_min, x_max, y_min, y_max);
fprintf('Expected single-truck footprint cells: %.1f\n', expected_cells);

%% --- Stage A: baseline (drivable parking lot = full rectangle) ---
% Parking lot polygon: x in [0,100], y in [-100,0] (matches X_MIN..X_MAX,
% Y_MIN..Y_MAX in map_const).  Closed rectangle, CCW.
boundary = [x_min y_min;
            x_max y_min;
            x_max y_max;
            x_min y_max];
base = generate_map(boundary);
n_base_off = sum(base(:) > 0);
fprintf('Stage A baseline: off-road cells = %d (expect ~0 for full-coverage polygon)\n', n_base_off);
okA = (n_base_off == 0);
fprintf('  baseline-free [%s]\n', ternary(okA, 'OK', 'FAIL'));

% Degenerate boundary (e.g., only 2 vertices) -> everything occupied.
base_bad = generate_map([0 0; 100 -100]);
okA2 = (sum(base_bad(:) > 0) == N*N);
fprintf('  degenerate polygon -> all occupied [%s]\n', ternary(okA2, 'OK', 'FAIL'));

%% --- Case 1: empty traffic on free baseline ---
empty_info = zeros(21, 1);
occ = add_obstacle(base, empty_info, traffic_size);
n1 = sum(occ(:) > 0);
ok1 = (n1 == 0);
fprintf('Case 1 (empty traffic, free baseline): occupied = %d  [%s]\n', n1, ternary(ok1, 'OK', 'FAIL'));

%% --- Case 2: single truck, yaw sweep ---
yaws = [0, pi/4, pi/2, pi, -pi/2];
fprintf('Case 2 (single truck @ (50,-50), yaw sweep):\n');
ok2 = true;
for k = 1:length(yaws)
    info = zeros(21, 1);
    info(1) = 50;
    info(2) = -50;
    info(3) = yaws(k);
    occ = add_obstacle(base, info, traffic_size);
    n = sum(occ(:) > 0);
    tol = 80;
    pass = abs(n - expected_cells) <= tol;
    fprintf('  yaw=%+6.2f rad : cells=%4d  expected~%4.0f  diff=%+4d  [%s]\n', ...
        yaws(k), n, expected_cells, round(n-expected_cells), ternary(pass, 'OK', 'FAIL'));
    if ~pass; ok2 = false; end
end

%% --- Case 3: dummy slots interleaved with real ---
info3 = zeros(21, 1);
info3(1) = 50;    info3(2) = -50;   info3(3) = 0;       % real (slot 1)
% slots 2..6 left zero (dummy, must be skipped)
info3(19) = 30;   info3(20) = -30;  info3(21) = pi/2;   % real (slot 7)
occ = add_obstacle(base, info3, traffic_size);
n3 = sum(occ(:) > 0);
ok3 = (abs(n3 - 2*expected_cells) <= 160);
fprintf('Case 3 (2 real + 5 dummy): cells=%d expected~%.0f  [%s]\n', ...
    n3, 2*expected_cells, ternary(ok3, 'OK', 'FAIL'));

%% --- Case 4: full Day4_5 Scenario 1 traffic (T01..T07) ---
trucks_deg = [40 -12  90;
              40 -24  90;
              40 -36  90;
              40 -48  90;
              39 -50   0;
              51 -50   0;
              63 -50   0];
info4 = zeros(21, 1);
for k = 1:7
    info4((k-1)*3 + 1) = trucks_deg(k, 1);
    info4((k-1)*3 + 2) = trucks_deg(k, 2);
    info4((k-1)*3 + 3) = deg2rad(trucks_deg(k, 3));
end
occ4 = add_obstacle(base, info4, traffic_size);
n4 = sum(occ4(:) > 0);
fprintf('Case 4 (Day4_5 trucks 7): cells=%d expected~%.0f\n', n4, 7*expected_cells);
ok4 = n4 > 5*expected_cells && n4 < 9*expected_cells;
fprintf('  occupancy plausible [%s]\n', ternary(ok4, 'OK', 'FAIL'));

t00_x = 76.1; t00_y = -5;
[r0, c0] = world_to_grid(t00_x, t00_y, c);
clean_T00 = (occ4(r0, c0) == 0);
fprintf('  cell @ T00 (76.1,-5) -> row=%d col=%d occ=%d  [%s]\n', ...
    r0, c0, occ4(r0, c0), ternary(clean_T00, 'OK', 'FAIL'));

%% --- Case 5: shrunken polygon -> walls show up in occupancy ---
% Drivable area = rectangle inset by 10 m on each side.  Outside should be 1.
inset = [10 -90; 90 -90; 90 -10; 10 -10];
base_inset = generate_map(inset);
n5 = sum(base_inset(:) > 0);
ok5 = n5 > 0 && n5 < N*N;
fprintf('Case 5 (inset polygon baseline): off-road=%d  [%s]\n', n5, ternary(ok5, 'OK', 'FAIL'));

%% --- Visualisations ---
% (a) baseline-only (full rectangle)
fig0 = figure('Visible','off','Position',[50 50 700 700]);
imagesc([x_min x_max], [y_max y_min], base); colormap(flipud(gray)); axis xy equal tight; grid on;
title('generate_map baseline (full parking-lot rectangle)');
xlabel('x [m]'); ylabel('y [m]');
saveas(fig0, fullfile(figs, 'generate_map_baseline.png'));
close(fig0);

% (b) inset baseline showing wall
fig0b = figure('Visible','off','Position',[50 50 700 700]);
imagesc([x_min x_max], [y_max y_min], base_inset); colormap(flipud(gray)); axis xy equal tight; grid on;
title('generate_map inset polygon — off-road wall visible');
xlabel('x [m]'); ylabel('y [m]');
saveas(fig0b, fullfile(figs, 'generate_map_inset.png'));
close(fig0b);

% (c) single truck on free baseline (yaw=pi/2)
info_v = zeros(21, 1); info_v(1)=50; info_v(2)=-50; info_v(3)=pi/2;
occ_v = add_obstacle(base, info_v, traffic_size);
fig1 = figure('Visible','off','Position',[50 50 700 700]);
imagesc([x_min x_max], [y_max y_min], occ_v); colormap(flipud(gray)); axis xy equal tight; grid on;
hold on; plot(50, -50, 'r+', 'MarkerSize', 14, 'LineWidth', 2);
title('Single truck (50,-50) yaw=pi/2 — rear-bumper anchored');
xlabel('x [m]'); ylabel('y [m]');
saveas(fig1, fullfile(figs, 'add_obstacle_single.png'));
close(fig1);

% (d) full Day4_5 scenario
fig2 = figure('Visible','off','Position',[50 50 700 700]);
imagesc([x_min x_max], [y_max y_min], occ4); colormap(flipud(gray)); axis xy equal tight; grid on;
hold on;
plot(0, -20, 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(t00_x, t00_y, 'b*', 'MarkerSize', 14, 'LineWidth', 2);
plot(80, -3, 'r*', 'MarkerSize', 14, 'LineWidth', 2);
for k = 1:7
    plot(trucks_deg(k,1), trucks_deg(k,2), 'm+', 'MarkerSize', 10, 'LineWidth', 1.5);
end
legend({'occ','ego start','T00 goal','legacy finish','truck rear-bumper'}, 'Location','southwest');
title('Day4_5 Scenario 1 — 7 trucks, T00 free, 200x200 / 0.5 m');
xlabel('x [m]'); ylabel('y [m]');
saveas(fig2, fullfile(figs, 'add_obstacle_scenario1.png'));
close(fig2);

fprintf('Saved figs/generate_map_baseline.png\n');
fprintf('Saved figs/generate_map_inset.png\n');
fprintf('Saved figs/add_obstacle_single.png\n');
fprintf('Saved figs/add_obstacle_scenario1.png\n');

%% --- Summary ---
all_ok = okA && okA2 && ok1 && ok2 && ok3 && ok4 && clean_T00 && ok5;
fprintf('\n--- RESULT: %s ---\n', ternary(all_ok, 'ALL PASS', 'FAIL'));
if ~all_ok
    error('test_add_obstacle: one or more cases failed.');
end

end

function out = ternary(cond, a, b)
if cond; out = a; else; out = b; end
end

function [row, col] = world_to_grid(x, y, c)
res = c.RES;
col = floor((x - c.X_MIN) / res) + 1;
row = floor((c.Y_MAX - y) / res) + 1;
if row < 1; row = 1; end
if row > double(c.N); row = double(c.N); end
if col < 1; col = 1; end
if col > double(c.N); col = double(c.N); end
end
