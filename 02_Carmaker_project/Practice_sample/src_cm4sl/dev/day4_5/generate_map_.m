function mapMatrix = generate_map_(map_boundary, traffic_info, traffic_size)
%GENERATE_MAP_  Self-contained baseline drivable-area mask for the
%   Day4_5_Scenario_1.slx "MATLAB Function1" block.
%
%   mapMatrix = generate_map_(map_boundary, traffic_info, traffic_size)
%
%   This file mirrors the chart script verbatim so the .slx is independent
%   of the rest of dev/day4_5/.  traffic_info / traffic_size are kept in the
%   signature only to preserve the existing inport wiring; they are unused.
%
%#codegen

u1 = traffic_info(1) * 0;   %#ok<NASGU>  % keep input in compile graph
u2 = traffic_size(1) * 0;   %#ok<NASGU>

mapMatrix = double(generate_map_local(map_boundary));
end

%% =====================================================================
%% Local helper functions (inlined from generate_map.m + map_const.m)
%% =====================================================================

function base_map = generate_map_local(map_boundary)
%#codegen
c = map_const_local();
N = c.N;
res = c.RES;
x_min = c.X_MIN;
y_max = c.Y_MAX;

MAX_BOUND = int32(16);

bx = zeros(MAX_BOUND, 1);
by = zeros(MAX_BOUND, 1);
M  = int32(0);

ne = int32(numel(map_boundary));
r  = int32(size(map_boundary, 1));
co = int32(size(map_boundary, 2));

if co == int32(2) && r >= int32(3)
    M = r;
    if M > MAX_BOUND; M = MAX_BOUND; end
    for i = int32(1):M
        bx(i) = map_boundary(i, 1);
        by(i) = map_boundary(i, 2);
    end
elseif r == int32(2) && co >= int32(3)
    M = co;
    if M > MAX_BOUND; M = MAX_BOUND; end
    for i = int32(1):M
        bx(i) = map_boundary(1, i);
        by(i) = map_boundary(2, i);
    end
elseif ne >= int32(6) && mod(double(ne), 2) == 0
    M = ne / int32(2);
    if M > MAX_BOUND; M = MAX_BOUND; end
    flat = map_boundary(:);
    for i = int32(1):M
        bx(i) = flat(2*i - 1);
        by(i) = flat(2*i);
    end
end

if M < 3
    base_map = ones(N, N, 'uint8');
    return;
end

% CCW sort around centroid so a Mux-supplied vertex order can't form a
% self-intersecting "bowtie" polygon.
cx_b = 0.0;
cy_b = 0.0;
for i = int32(1):M
    cx_b = cx_b + bx(i);
    cy_b = cy_b + by(i);
end
cx_b = cx_b / double(M);
cy_b = cy_b / double(M);

ang = zeros(MAX_BOUND, 1);
for i = int32(1):M
    ang(i) = atan2(by(i) - cy_b, bx(i) - cx_b);
end

for i = int32(1):M-int32(1)
    min_k = i;
    for j = i+int32(1):M
        if ang(j) < ang(min_k)
            min_k = j;
        end
    end
    if min_k ~= i
        tmp = ang(i);  ang(i) = ang(min_k);  ang(min_k) = tmp;
        tmp = bx(i);   bx(i)  = bx(min_k);   bx(min_k)  = tmp;
        tmp = by(i);   by(i)  = by(min_k);   by(min_k)  = tmp;
    end
end

base_map = zeros(N, N, 'uint8');

inflate_m = c.EGO_W * 0.5 + c.SAFETY_MARGIN;

for row = 1:N
    wy = y_max - (double(row) - 0.5) * res;
    for col = 1:N
        wx = x_min + (double(col) - 0.5) * res;
        if ~point_in_polygon(wx, wy, bx, by, M)
            base_map(row, col) = uint8(1);
        else
            if dist_to_polygon_edge(wx, wy, bx, by, M) < inflate_m
                base_map(row, col) = uint8(1);
            end
        end
    end
end
end

function c = map_const_local()
%#codegen
c.N        = int32(200);
c.RES      = 0.5;
c.X_MIN    = 0.0;
c.X_MAX    = 100.0;
c.Y_MIN    = -100.0;
c.Y_MAX    = 0.0;
c.TRUCK_W  = 2.48;
c.TRUCK_L  = 11.5;
c.EGO_W    = 1.9;
c.EGO_L    = 4.7;
c.WHEELBASE = 2.8;
c.SAFETY_MARGIN = 0.8;
c.CLEAR_MAX = 3.0;
c.W_CLEAR   = 1.2;
c.PARK_BOX_L = 6.0;
c.PARK_BOX_W = 2.3;       % was 3.0 — tightened to cap yaw error at ~2.4 deg
c.PARK_TOL   = 0.05;
end

function d_min = dist_to_polygon_edge(px, py, bx, by, M)
%#codegen
d_min = 1.0e9;
j = double(M);
for i = 1:double(M)
    d_seg = point_to_segment(px, py, bx(j), by(j), bx(i), by(i));
    if d_seg < d_min
        d_min = d_seg;
    end
    j = i;
end
end

function d = point_to_segment(px, py, ax, ay, qx, qy)
%#codegen
vx = qx - ax;
vy = qy - ay;
wx = px - ax;
wy = py - ay;
v2 = vx*vx + vy*vy;
if v2 < 1.0e-9
    d = hypot(px - ax, py - ay);
    return;
end
t = (wx*vx + wy*vy) / v2;
if t < 0.0
    t = 0.0;
elseif t > 1.0
    t = 1.0;
end
cx = ax + t * vx;
cy = ay + t * vy;
d = hypot(px - cx, py - cy);
end

function inside = point_in_polygon(px, py, bx, by, M)
%#codegen
inside = false;
j = double(M);
for i = 1:double(M)
    xi = bx(i); yi = by(i);
    xj = bx(j); yj = by(j);
    if ((yi > py) ~= (yj > py))
        x_at = (xj - xi) * (py - yi) / (yj - yi + eps) + xi;
        if px < x_at
            inside = ~inside;
        end
    end
    j = i;
end
end
