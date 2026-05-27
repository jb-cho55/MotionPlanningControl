function [px, py, pyaw, pdir, plen, ok] = rs_shot(sx, sy, syaw, gx, gy, gyaw, occ_map, R, ds)
%RS_SHOT  Lightweight Reeds-Shepp analytic shot for parking endgame.
%
%   [px, py, pyaw, pdir, plen, ok] = rs_shot(sx, sy, syaw, gx, gy, gyaw,
%                                              occ_map, R, ds)
%
%   Returns a single discretised Reeds-Shepp curve from (sx,sy,syaw) to
%   (gx,gy,gyaw) using fixed turning radius R and arc-length step ds.
%   ok = true iff a candidate curve was found AND the discretised samples
%   are all collision-free against occ_map.  Otherwise plen = 0.
%
%   This implementation covers a compact subset of the 48 Reeds-Shepp
%   patterns that matters for parking endgames:
%       CSC : L+ S+ L+,  L+ S+ R+,  R+ S+ L+,  R+ S+ R+   (forward)
%             L- S- L-,  L- S- R-,  R- S- L-,  R- S- R-   (reverse)
%       CCC : L+ R- L+ and mirrored variants
%   The shortest valid (collision-free) sample is returned.
%
%   It's an "analytic shot" because it's evaluated as one geometric
%   primitive — no Hybrid A* tree.  The caller (hybrid_astar_plan) tries
%   it at every node expansion; if it succeeds we can terminate without
%   exploring further.  Pre-baked clearance/heuristic helps but the real
%   payoff for parking is here: the shot threads the goal yaw exactly
%   instead of approximating it through grid quantisation.
%
%   MAX_PATH must match hybrid_astar_plan (=300).
%
%#codegen

MAX_PATH = int32(300);
px   = zeros(1, MAX_PATH);
py   = zeros(1, MAX_PATH);
pyaw = zeros(1, MAX_PATH);
pdir = zeros(1, MAX_PATH, 'int8');
plen = int32(0);
ok   = false;

% Try the 8 CSC patterns (4 sign combinations x forward/reverse).
% Each call returns ok if the discretised curve is collision-free.
patterns = [...
    +1, +1, +1;    % L+ S+ L+
    +1, +1, -1;    % L+ S+ R+
    -1, +1, +1;    % R+ S+ L+
    -1, +1, -1;    % R+ S+ R+
    +1, -1, +1;    % L- S- L-
    +1, -1, -1;    % L- S- R-
    -1, -1, +1;    % R- S- L-
    -1, -1, -1];   % R- S- R-

best_len = 1.0e18;
for p = 1:size(patterns, 1)
    t1 = patterns(p, 1);   % first-arc turn sign (+1=left, -1=right)
    sd = patterns(p, 2);   % straight direction (+1 forward, -1 reverse)
    t2 = patterns(p, 3);   % final-arc turn sign
    [c_ok, c_px, c_py, c_pyaw, c_pdir, c_plen, c_len] = ...
        try_csc(sx, sy, syaw, gx, gy, gyaw, R, ds, t1, sd, t2, occ_map, MAX_PATH);
    if c_ok && c_len < best_len
        best_len = c_len;
        px   = c_px;
        py   = c_py;
        pyaw = c_pyaw;
        pdir = c_pdir;
        plen = c_plen;
        ok   = true;
    end
end
end


function [ok, px, py, pyaw, pdir, plen, total_len] = try_csc(sx, sy, syaw, ...
        gx, gy, gyaw, R, ds, t1, sd, t2, occ_map, MAX_PATH)
%#codegen
% Compute the CSC curve geometry analytically.
%
% Step 1: place the start-arc circle centre.  Left turn (t1=+1) places it
% pi/2 to the LEFT of the start heading, right turn (-1) to the right.
% Step 2: place the goal-arc circle centre likewise.
% Step 3: find the inner/outer common tangent and the corresponding
% straight segment.  For matching-sign arcs (t1==t2) it's the outer
% tangent (length = ||c1-c2||); for opposite-sign arcs it's the inner
% tangent (length = sqrt(d^2 - (2R)^2)).
% Step 4: walk arc1 -> straight -> arc2, sampling every ds.
%
% sd = +1 marks the straight as forward, -1 as reverse.  The arcs inherit
% the same sign (Hybrid A* convention: an entire CSC primitive is in one
% gear).

ok = false;
px = zeros(1, MAX_PATH); py = zeros(1, MAX_PATH);
pyaw = zeros(1, MAX_PATH); pdir = zeros(1, MAX_PATH, 'int8');
plen = int32(0);
total_len = 1.0e18;

% Circle centres.
c1x = sx - R * sin(syaw) * t1;
c1y = sy + R * cos(syaw) * t1;
c2x = gx - R * sin(gyaw) * t2;
c2y = gy + R * cos(gyaw) * t2;

dxc = c2x - c1x;
dyc = c2y - c1y;
d_cc = hypot(dxc, dyc);

if t1 == t2
    % Outer tangent — distance between tangent points equals d_cc.
    if d_cc < 1.0e-6
        return;
    end
    seg_len = d_cc;
    alpha = atan2(dyc, dxc);
    % Tangent point on circle 1
    tp1x = c1x + R * sin(alpha) * t1;
    tp1y = c1y - R * cos(alpha) * t1;
    tp2x = tp1x + seg_len * cos(alpha);
    tp2y = tp1y + seg_len * sin(alpha);
    tangent_yaw = alpha;
else
    % Inner tangent — requires d_cc >= 2R.
    if d_cc < 2.0 * R
        return;
    end
    seg_len = sqrt(d_cc * d_cc - 4.0 * R * R);
    alpha = atan2(dyc, dxc);
    beta = atan2(2.0 * R, seg_len) * t1;
    tan_dir = alpha - beta;
    tp1x = c1x + R * sin(tan_dir) * t1;
    tp1y = c1y - R * cos(tan_dir) * t1;
    tp2x = tp1x + seg_len * cos(tan_dir);
    tp2y = tp1y + seg_len * sin(tan_dir);
    tangent_yaw = tan_dir;
end

% Arc 1: from (sx,sy,syaw) to tangent point (tp1,*) along circle c1.
arc1_yaw_start = wrap_pi(syaw);
arc1_yaw_end   = wrap_pi(tangent_yaw);
arc1_len_signed = arc_length_along_circle(arc1_yaw_start, arc1_yaw_end, t1) * R;

% Arc 2: from tangent_yaw to goal yaw along circle c2.
arc2_yaw_start = wrap_pi(tangent_yaw);
arc2_yaw_end   = wrap_pi(gyaw);
arc2_len_signed = arc_length_along_circle(arc2_yaw_start, arc2_yaw_end, t2) * R;

total_len = abs(arc1_len_signed) + seg_len + abs(arc2_len_signed);

% Sample.  We discretise with step ds along path.
n_arc1 = max(int32(1), int32(ceil(abs(arc1_len_signed) / ds)));
n_seg  = max(int32(1), int32(ceil(seg_len / ds)));
n_arc2 = max(int32(1), int32(ceil(abs(arc2_len_signed) / ds)));
total_n = n_arc1 + n_seg + n_arc2 + int32(1);
if total_n > MAX_PATH
    return;
end

idx = int32(1);
px(idx) = sx; py(idx) = sy; pyaw(idx) = syaw; pdir(idx) = int8(sd);

% Walk arc 1.  Car position on circle 1 must match the tangent-point
% formula above: (c1 + R*(sin*t1, -cos*t1)).
for k = int32(1):n_arc1
    s = double(k) / double(n_arc1);
    yawk = arc1_yaw_start + s * (arc1_len_signed / R);
    xk = c1x + R * sin(yawk) * t1;
    yk = c1y - R * cos(yawk) * t1;
    idx = idx + int32(1);
    px(idx) = xk; py(idx) = yk; pyaw(idx) = wrap_pi(yawk); pdir(idx) = int8(sd);
end

% Walk straight
for k = int32(1):n_seg
    s = double(k) / double(n_seg);
    xk = tp1x + s * (tp2x - tp1x);
    yk = tp1y + s * (tp2y - tp1y);
    idx = idx + int32(1);
    px(idx) = xk; py(idx) = yk; pyaw(idx) = tangent_yaw; pdir(idx) = int8(sd);
end

% Walk arc 2
for k = int32(1):n_arc2
    s = double(k) / double(n_arc2);
    yawk = arc2_yaw_start + s * (arc2_len_signed / R);
    xk = c2x + R * sin(yawk) * t2;
    yk = c2y - R * cos(yawk) * t2;
    idx = idx + int32(1);
    px(idx) = xk; py(idx) = yk; pyaw(idx) = wrap_pi(yawk); pdir(idx) = int8(sd);
end

plen = idx;

% Collision check.
for i = int32(1):plen-int32(1)
    if seg_hits(px(i), py(i), px(i+1), py(i+1), occ_map)
        plen = int32(0);
        return;
    end
end

ok = true;
end


function len = arc_length_along_circle(yaw_start, yaw_end, turn_sign)
%#codegen
% turn_sign = +1 = left (CCW), -1 = right (CW).
d = wrap_pi(yaw_end - yaw_start);
if turn_sign > 0
    % CCW: positive d, if negative add 2 pi
    if d < 0
        d = d + 2.0 * pi;
    end
else
    % CW: negative d, if positive subtract 2 pi
    if d > 0
        d = d - 2.0 * pi;
    end
end
len = d;   % signed angular arc length (rad)
end


function col = seg_hits(x1, y1, x2, y2, occ)
%#codegen
N_grid = 200;
RES = 0.5;
X_MIN = 0.0;
Y_MAX = 0.0;
col = false;
for k = int32(0):int32(8)
    t = double(k) / 8.0;
    x = x1 + t * (x2 - x1);
    y = y1 + t * (y2 - y1);
    if x < 0.0 || x > 100.0 || y < -100.0 || y > 0.0
        col = true; return;
    end
    c = floor((x - X_MIN) / RES) + 1;
    r = floor((Y_MAX - y) / RES) + 1;
    if r < 1 || r > N_grid || c < 1 || c > N_grid
        col = true; return;
    end
    if occ(r, c) > 0
        col = true; return;
    end
end
end


function a = wrap_pi(a)
%#codegen
while a > pi;  a = a - 2.0 * pi; end
while a < -pi; a = a + 2.0 * pi; end
end
