function analyze_carmaker_run(mat_dir)
%ANALYZE_CARMAKER_RUN  Visualise a Day4_5 Scenario 1 CarMaker run from the
%   .mat files the .slx To File blocks drop next to the simulation.
%
%   analyze_carmaker_run()              % auto-find latest matching folder
%   analyze_carmaker_run(mat_dir)       % explicit dir holding the .mat files
%
%   Expected .mat files (produced by Day4_5_Scenario_1.slx To File blocks):
%       car_fr1_log.mat     - var car_fr1_log [5 x N]: [t; tx; ty; rz; vx]
%       control_log.mat     - var control_log  [5 x N]: [t; steer_fl; steer_fr; ax; sel]
%       last_occ.mat        - var occ_map      [200 x 200] with obstacles baked in
%       last_path_x.mat     - var path_x_log   [300 x N], last column = final path
%       last_path_y.mat     - var path_y_log   [300 x N]
%       last_path_len.mat   - var path_len_log [1 x N], last value = final length
%
%   Renders (next to the .mat files):
%       <stamp>__trajectory.png  - occ + ego trajectory + planned path overlay
%       <stamp>__signals.png     - v, ax, steer, selector_ctrl, dist-to-T00
%
%   No dependencies on dev/day4_5 helpers — occupancy grid is loaded from
%   last_occ.mat as-is (with obstacles baked in by add_obstacle_).

here = fileparts(mfilename('fullpath'));
addpath(here);

if nargin < 1 || isempty(mat_dir)
    mat_dir = find_latest_mat_dir();
end
if ~exist(mat_dir, 'dir')
    error('mat_dir not found: %s', mat_dir);
end

fprintf('=== analyze_carmaker_run ===\n');
fprintf('Reading .mat files from: %s\n', mat_dir);

% --- Load logs ---
[car_fr1_log, has_ego]   = load_var(mat_dir, 'car_fr1_log.mat',    'car_fr1_log');
[control_log, has_ctl]   = load_var(mat_dir, 'control_log.mat',    'control_log');
[occ_map,     has_occ]   = load_var(mat_dir, 'last_occ.mat',       'occ_map');
[path_x_log,  has_px]    = load_var(mat_dir, 'last_path_x.mat',    'path_x_log');
[path_y_log,  has_py]    = load_var(mat_dir, 'last_path_y.mat',    'path_y_log');
[path_len_log,has_pl]    = load_var(mat_dir, 'last_path_len.mat',  'path_len_log');

if ~has_ego
    error('car_fr1_log.mat not found — run the simulation first.');
end

% Constants for the grid (mirrors Parking.m map_const_local)
N = 200; RES = 0.5;
X_MIN = 0; X_MAX = 100; Y_MIN = -100; Y_MAX = 0;
EGO_W = 1.9;

% --- Parse car_fr1_log ---
t      = car_fr1_log(1, :)';
ego_x  = car_fr1_log(2, :)';
ego_y  = car_fr1_log(3, :)';
ego_yaw= car_fr1_log(4, :)';
ego_v  = car_fr1_log(5, :)';
n_samp = numel(t);

% --- Parse control_log if available ---
if has_ctl
    ct_t   = control_log(1, :)';
    sfl    = control_log(2, :)';
    sfr    = control_log(3, :)'; %#ok<NASGU>  (= sfl in current model)
    ax_cmd = control_log(4, :)';
    sel    = control_log(5, :)';
else
    ct_t = t; sfl = nan(n_samp,1); ax_cmd = nan(n_samp,1); sel = nan(n_samp,1);
end

% --- Final path snapshot ---
% To File "Array" stores [N+1 x T] with row 1 = time.  Strip the time row.
if has_px && has_py && has_pl
    px_buf = strip_time_row(path_x_log);
    py_buf = strip_time_row(path_y_log);
    plen_buf = strip_time_row(path_len_log);
    if size(plen_buf, 2) >= 1
        plen_final = double(plen_buf(1, end));
    else
        plen_final = numel(px_buf);
    end
    if size(px_buf, 2) >= 1
        px_final = px_buf(:, end);
        py_final = py_buf(:, end);
    else
        px_final = px_buf(:); py_final = py_buf(:);
    end
    if plen_final >= 2 && plen_final <= numel(px_final)
        px_final = px_final(1:plen_final);
        py_final = py_final(1:plen_final);
    end
else
    plen_final = 0; px_final = []; py_final = [];
end

t00_x = 76.1;  t00_y = -5;
d_to_t00 = hypot(ego_x - t00_x, ego_y - t00_y);

% --- Summary ---
fprintf('\n--- Summary ---\n');
fprintf('Samples: %d   sim duration: %.2f s\n', n_samp, t(end) - t(1));
fprintf('Ego start: (%.2f, %.2f)  end: (%.2f, %.2f)\n', ego_x(1), ego_y(1), ego_x(end), ego_y(end));
fprintf('Min |ego - T00|: %.2f m   final: %.2f m\n', min(d_to_t00), d_to_t00(end));
fprintf('Final yaw error from pi/2: %.2f deg\n', rad2deg(angdiff(ego_yaw(end), pi/2)));
if has_occ
    fprintf('Occupancy grid: %s, occupied=%d (with obstacles)\n', mat2str(size(occ_map)), sum(occ_map(:) > 0));
end
if has_pl
    fprintf('Final path length: %d samples\n', plen_final);
end

% --- Output paths (always under dev/day4_5/figs) ---
figs_dir = fullfile(here, 'figs');
if ~exist(figs_dir, 'dir'); mkdir(figs_dir); end
stamp = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<DATST>
prefix = fullfile(figs_dir, ['run_' stamp]);

% --- Trajectory overlay on occupancy map (with obstacles) ---
fig = figure('Visible','off','Position',[50 50 950 950]);
if has_occ
    imagesc([X_MIN X_MAX], [Y_MAX Y_MIN], occ_map);
    colormap(flipud(gray)); axis xy equal tight; grid on; hold on;
else
    axis equal; grid on; hold on;
end
% Planned path
if ~isempty(px_final)
    plot(px_final, py_final, 'c-', 'LineWidth', 1.0);
end
% Ego trajectory
plot(ego_x, ego_y, 'b-', 'LineWidth', 1.6);
plot(ego_x(1),  ego_y(1),  'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(ego_x(end),ego_y(end),'b+', 'MarkerSize', 12, 'LineWidth', 2);
plot(t00_x, t00_y, 'r*', 'MarkerSize', 14, 'LineWidth', 2);
xlim([X_MIN X_MAX]); ylim([Y_MIN Y_MAX]);
xlabel('x [m]'); ylabel('y [m]');
title(sprintf('Day4_5 Scenario 1 — d_{end}=%.2f m, path_{len}=%d', ...
    d_to_t00(end), plen_final));
lgd = {};
if ~isempty(px_final); lgd{end+1} = 'planned path'; end
lgd = [lgd, {'ego','start','end','T00'}];
legend(lgd, 'Location', 'southwest');
saveas(fig, [prefix '__trajectory.png']);
close(fig);

% --- Signals ---
fig2 = figure('Visible','off','Position',[50 50 1200 1000]);
subplot(5,1,1); plot(t, ego_v, 'r-'); grid on;
title('ego v [m/s] (Car.vx)'); xlabel('t [s]');
subplot(5,1,2); plot(ct_t, ax_cmd, 'b-'); grid on;
title('desired ax [m/s^2]'); xlabel('t [s]');
subplot(5,1,3); plot(ct_t, sfl, 'k-'); grid on;
title('steer FL [rad]'); xlabel('t [s]');
subplot(5,1,4); plot(ct_t, sel, 'm-'); grid on; ylim([-1.5 1.5]);
title('DM.SelectorCtrl (+1 drive, -1 reverse)'); xlabel('t [s]');
subplot(5,1,5); plot(t, d_to_t00, 'g-'); grid on;
title('distance to T00 [m]'); xlabel('t [s]');
saveas(fig2, [prefix '__signals.png']);
close(fig2);

fprintf('\nSaved figures:\n  %s__trajectory.png\n  %s__signals.png\n', prefix, prefix);

end

function p = find_latest_mat_dir()
% Look for car_fr1_log.mat under the project root, return its folder.
root_candidates = {
    'C:\Users\gmkk6\Desktop\MotionPlanningControl\02_Carmaker_project\Practice_sample';
    'C:\Users\gmkk6\Desktop\MotionPlanningControl\02_Carmaker_project';
    pwd();
};
hits = struct('folder', {}, 'datenum', {});
for k = 1:numel(root_candidates)
    root = root_candidates{k};
    if ~exist(root, 'dir'); continue; end
    d = dir(fullfile(root, '**', 'car_fr1_log.mat'));
    for i = 1:numel(d)
        hits(end+1).folder = d(i).folder; %#ok<AGROW>
        hits(end).datenum  = d(i).datenum;
    end
end
if isempty(hits)
    error('No car_fr1_log.mat found under MotionPlanningControl/02_Carmaker_project/.\n   Run the simulation first.');
end
[~, idx] = max([hits.datenum]);
p = hits(idx).folder;
end

function [v, ok] = load_var(mat_dir, fname, vname)
v  = [];
ok = false;
fpath = fullfile(mat_dir, fname);
if ~exist(fpath, 'file')
    fprintf('  [skip] %s not found\n', fname);
    return;
end
S = load(fpath);
raw = [];
if isfield(S, vname)
    raw = S.(vname);
else
    flds = fieldnames(S);
    if ~isempty(flds); raw = S.(flds{1}); end
end
if isempty(raw); return; end

% Timeseries-format To File yields a `timeseries` object.  The time axis
% is whichever Data dim matches numel(.Time).  Pull the LAST frame.
if isa(raw, 'timeseries')
    d  = raw.Data;
    nt = numel(raw.Time);
    sz = size(d);
    time_dim = find(sz == nt, 1, 'last');   % prefer last matching dim
    if isempty(time_dim); time_dim = ndims(d); end
    % Index "last frame" along time_dim
    idx = repmat({':'}, 1, ndims(d));
    idx{time_dim} = sz(time_dim);
    v = squeeze(d(idx{:}));
    fprintf('  loaded %s (timeseries, T=%d -> last frame %s)\n', fname, nt, mat2str(size(v)));
else
    v = raw;
    fprintf('  loaded %s (%s, size %s)\n', fname, vname, mat2str(size(v)));
end
ok = true;
end

function out = strip_time_row(arr)
% To File "Array" format stores [N+1 x T] with row 1 = sim time.
% Drop row 1 unless the array is too short to have signal rows.
if isempty(arr); out = arr; return; end
if size(arr, 1) >= 2
    out = arr(2:end, :);
else
    out = arr;
end
end

function d = angdiff(a, b)
d = a - b;
while d > pi;  d = d - 2*pi; end
while d < -pi; d = d + 2*pi; end
end
