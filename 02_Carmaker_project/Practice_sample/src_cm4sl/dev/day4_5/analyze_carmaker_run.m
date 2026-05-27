function analyze_carmaker_run(erg_path)
%ANALYZE_CARMAKER_RUN  Post-process a Day4_5 Scenario 1 CarMaker run.
%
%   analyze_carmaker_run()              % auto-find latest day4_5_scenario1
%   analyze_carmaker_run(erg_path)      % full path to .erg file
%
%   Reads the ERG (CarMaker binary log) + its .erg.info sidecar, extracts:
%       - Ego pose:   Car.Fr1.tx, Car.Fr1.ty, Car.Fr1.rz
%       - Ego speed:  Car.vx
%       - Steering:   Car.CFL.rz_ext, Car.CFR.rz_ext
%       - Accel cmd:  AccelCtrl.DesiredAx
%   Re-builds the static occupancy grid (Day4_5 trucks T01..T07) and
%   produces a comparable visualisation to test_pipeline.m so the
%   .m-only closed-loop and the .slx + CarMaker run can be compared
%   side by side.
%
%   Outputs (saved next to the .erg):
%       <run>__trajectory.png   - ego path over occupancy
%       <run>__signals.png      - v, ax, steer time series
%       <run>__clearance.png    - min distance to nearest truck vs t
%   Console:
%       - sim duration, final position, distance to T00, min clearance,
%         collision flag (clearance < 0).

here = fileparts(mfilename('fullpath'));
addpath(here);

if nargin < 1 || isempty(erg_path)
    erg_path = find_latest_run();
end
if ~exist(erg_path, 'file')
    error('ERG file not found: %s', erg_path);
end
info_path = [erg_path '.info'];
if ~exist(info_path, 'file')
    error('Info file not found: %s', info_path);
end

fprintf('=== analyze_carmaker_run ===\n');
fprintf('ERG: %s\n', erg_path);

% --- Parse .erg.info to learn quantity layout ---
[qty_names, qty_types] = parse_erg_info(info_path);
fprintf('Quantities in log: %d\n', length(qty_names));

% --- Read ERG binary ---
data = read_erg(erg_path, qty_types);
n_samples = size(data, 1);
fprintf('Samples: %d\n', n_samples);

% --- Pull the signals we need ---
t      = pick(data, qty_names, 'Time');                  if isempty(t);  t = (0:n_samples-1)' * 0.01; end
ego_x  = pick(data, qty_names, 'Car.Fr1.tx');
ego_y  = pick(data, qty_names, 'Car.Fr1.ty');
ego_yaw= pick(data, qty_names, 'Car.Fr1.rz');
ego_v  = pick(data, qty_names, 'Car.vx');
steer  = pick(data, qty_names, 'Car.CFL.rz_ext');
ax_cmd = pick(data, qty_names, 'AccelCtrl.DesiredAx');

if isempty(ego_x) || isempty(ego_y)
    error('Required quantity Car.Fr1.tx/ty not found in log.');
end

% --- Build static occupancy for visualisation (same as test_pipeline) ---
c = map_const();
boundary = [c.X_MIN c.Y_MIN; c.X_MAX c.Y_MIN; c.X_MAX c.Y_MAX; c.X_MIN c.Y_MAX];
base = generate_map(boundary);
trucks_deg = [40 -12  90;
              40 -24  90;
              40 -36  90;
              40 -48  90;
              39 -50   0;
              51 -50   0;
              63 -50   0];
info = zeros(21, 1);
for k = 1:7
    info((k-1)*3 + 1) = trucks_deg(k, 1);
    info((k-1)*3 + 2) = trucks_deg(k, 2);
    info((k-1)*3 + 3) = deg2rad(trucks_deg(k, 3));
end
occ = add_obstacle(base, info, [c.TRUCK_W, c.TRUCK_L]);

% --- Compute clearance over time (ego cell -> nearest occupied cell) ---
clear_map = compute_clearance(occ);
clr = nan(n_samples, 1);
for k = 1:n_samples
    if ego_x(k) < c.X_MIN || ego_x(k) > c.X_MAX || ego_y(k) < c.Y_MIN || ego_y(k) > c.Y_MAX
        clr(k) = -1;
        continue;
    end
    col = floor((ego_x(k) - c.X_MIN)/c.RES) + 1;
    row = floor((c.Y_MAX - ego_y(k))/c.RES) + 1;
    if row < 1 || row > double(c.N) || col < 1 || col > double(c.N)
        clr(k) = -1;
    else
        clr(k) = double(clear_map(int32(row), int32(col))) - c.EGO_W*0.5;
    end
end

t00_x = 76.1; t00_y = -5;
d_to_t00 = hypot(ego_x - t00_x, ego_y - t00_y);

% --- Summary ---
fprintf('\n--- Summary ---\n');
fprintf('Sim duration : %.2f s (n=%d samples)\n', t(end)-t(1), n_samples);
fprintf('Ego start    : (%.2f, %.2f)\n', ego_x(1), ego_y(1));
fprintf('Ego end      : (%.2f, %.2f)\n', ego_x(end), ego_y(end));
fprintf('Min |ego - T00|: %.2f m\n', min(d_to_t00));
fprintf('Min clearance (ego body to truck edge): %.2f m\n', min(clr));
if min(clr) < 0
    fprintf('!! COLLISION detected (clearance went negative)\n');
else
    fprintf('No collision (clearance > 0 throughout)\n');
end

% --- Output paths ---
[outdir, runname, ~] = fileparts(erg_path);
prefix = fullfile(outdir, runname);

% --- Trajectory plot ---
fig = figure('Visible','off','Position',[50 50 900 900]);
imagesc([c.X_MIN c.X_MAX], [c.Y_MAX c.Y_MIN], occ);
colormap(flipud(gray)); axis xy equal tight; grid on; hold on;
plot(ego_x, ego_y, 'b-', 'LineWidth', 1.6);
plot(ego_x(1), ego_y(1), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(t00_x, t00_y, 'r*', 'MarkerSize', 14, 'LineWidth', 2);
plot(ego_x(end), ego_y(end), 'b+', 'MarkerSize', 12, 'LineWidth', 2);
xlim([c.X_MIN c.X_MAX]); ylim([c.Y_MIN c.Y_MAX]);
legend({'occ','ego trajectory','start','T00','end'}, 'Location','southwest');
title(sprintf('CarMaker run — Day4_5 Scenario 1 (min clr=%.2f m, d_{end}=%.2f m)', ...
    min(clr), d_to_t00(end)));
xlabel('x [m]'); ylabel('y [m]');
saveas(fig, [prefix '__trajectory.png']);
close(fig);

% --- Signal plot ---
fig2 = figure('Visible','off','Position',[50 50 1200 800]);
n = 4;
subplot(n,1,1); plot(t, ego_v, 'r-'); grid on;
title('ego v [m/s] (Car.vx)'); xlabel('t [s]');
subplot(n,1,2); plot(t, ax_cmd, 'b-'); grid on;
title('desired ax [m/s^2] (AccelCtrl.DesiredAx)'); xlabel('t [s]');
subplot(n,1,3); plot(t, steer, 'k-'); grid on;
title('steer [rad] (Car.CFL.rz_ext)'); xlabel('t [s]');
subplot(n,1,4); plot(t, d_to_t00, 'm-'); grid on;
title('distance to T00 [m]'); xlabel('t [s]');
saveas(fig2, [prefix '__signals.png']);
close(fig2);

% --- Clearance plot ---
fig3 = figure('Visible','off','Position',[50 50 1100 500]);
plot(t, clr, 'b-', 'LineWidth', 1.2); hold on; grid on;
yline(0, 'r--', 'collision threshold');
yline(c.SAFETY_MARGIN, 'g--', sprintf('safety margin (%.2f m)', c.SAFETY_MARGIN));
title('Ego body clearance to nearest truck edge [m]');
xlabel('t [s]'); ylabel('clearance [m]');
saveas(fig3, [prefix '__clearance.png']);
close(fig3);

fprintf('\nSaved figures next to ERG:\n  %s__trajectory.png\n  %s__signals.png\n  %s__clearance.png\n', ...
    prefix, prefix, prefix);

end


function p = find_latest_run()
% Auto-find the most recent SimOutput .erg with "day4_5" or "Day4_5" in path.
root = 'C:\Users\gmkk6\Desktop\MotionPlanningControl\02_Carmaker_project\Practice_sample\SimOutput';
d = dir(fullfile(root, '**', '*.erg'));
keep = false(length(d), 1);
for i = 1:length(d)
    if contains(lower(d(i).folder), 'day4_5') || contains(lower(d(i).name), 'day4_5')
        keep(i) = true;
    end
end
d = d(keep);
if isempty(d)
    error(['No .erg with "day4_5" in path under %s.\n' ...
           'Run the scenario first, or pass the explicit .erg path.'], root);
end
[~, idx] = max([d.datenum]);
p = fullfile(d(idx).folder, d(idx).name);
end

function [names, types] = parse_erg_info(info_path)
% Parse Name/Type lines from .erg.info.
names = strings(0, 1);
types = strings(0, 1);
fid = fopen(info_path, 'rt');
if fid < 0; error('Cannot open %s', info_path); end
cur_name = '';
cur_type = '';
cur_idx  = 0;
tline = fgetl(fid);
while ischar(tline)
    tok = regexp(tline, 'File\.At\.(\d+)\.Name\s*=\s*(.+)', 'tokens', 'once');
    if ~isempty(tok)
        if cur_idx > 0 && ~isempty(cur_name)
            names(cur_idx, 1) = string(strtrim(cur_name));
            types(cur_idx, 1) = string(strtrim(cur_type));
        end
        cur_idx  = str2double(tok{1});
        cur_name = tok{2};
        cur_type = '';
    end
    tok = regexp(tline, 'File\.At\.(\d+)\.Type\s*=\s*(.+)', 'tokens', 'once');
    if ~isempty(tok)
        cur_type = tok{2};
    end
    tline = fgetl(fid);
end
if cur_idx > 0 && ~isempty(cur_name)
    names(cur_idx, 1) = string(strtrim(cur_name));
    types(cur_idx, 1) = string(strtrim(cur_type));
end
fclose(fid);
end

function data = read_erg(erg_path, types)
% Read a CarMaker ERG binary (little-endian, packed columns).
fid = fopen(erg_path, 'rb', 'ieee-le');
if fid < 0; error('Cannot open %s', erg_path); end
% ERG header (CarMaker 5+): 8-byte magic + 8 reserved.  Skip if present.
hdr = fread(fid, 16, 'uint8');
if numel(hdr) < 16
    fseek(fid, 0, 'bof');
end
rec_size = 0;
col_kinds = strings(length(types), 1);
for i = 1:length(types)
    t = lower(strtrim(char(types(i))));
    col_kinds(i) = t;
    switch t
        case {'float','single'};         rec_size = rec_size + 4;
        case {'double'};                 rec_size = rec_size + 8;
        case {'int','int32','long'};     rec_size = rec_size + 4;
        case {'uint','uint32','ulong'};  rec_size = rec_size + 4;
        case {'short','int16'};          rec_size = rec_size + 2;
        case {'ushort','uint16'};        rec_size = rec_size + 2;
        case {'char','int8'};            rec_size = rec_size + 1;
        case {'uchar','uint8'};          rec_size = rec_size + 1;
        otherwise;                        rec_size = rec_size + 4;
    end
end
% File body
start = ftell(fid);
fseek(fid, 0, 'eof');
nbytes = ftell(fid) - start;
fseek(fid, start, 'bof');
n_rec = floor(nbytes / rec_size);
raw = fread(fid, n_rec * rec_size, '*uint8');
fclose(fid);
data = zeros(n_rec, length(types));
off = 0;
for i = 1:length(types)
    t = col_kinds(i);
    switch t
        case {'float','single'};         sz = 4; fmt = 'single';
        case {'double'};                 sz = 8; fmt = 'double';
        case {'int','int32','long'};     sz = 4; fmt = 'int32';
        case {'uint','uint32','ulong'};  sz = 4; fmt = 'uint32';
        case {'short','int16'};          sz = 2; fmt = 'int16';
        case {'ushort','uint16'};        sz = 2; fmt = 'uint16';
        case {'char','int8'};            sz = 1; fmt = 'int8';
        case {'uchar','uint8'};          sz = 1; fmt = 'uint8';
        otherwise;                        sz = 4; fmt = 'single';
    end
    col = zeros(n_rec, 1);
    for r = 0:n_rec-1
        chunk = raw(r*rec_size + off + (1:sz));
        col(r+1) = typecast(uint8(chunk(:)).', fmt);
    end
    data(:, i) = col;
    off = off + sz;
end
end

function v = pick(data, names, q)
v = [];
idx = find(strcmpi(names, q), 1);
if ~isempty(idx)
    v = data(:, idx);
end
end
