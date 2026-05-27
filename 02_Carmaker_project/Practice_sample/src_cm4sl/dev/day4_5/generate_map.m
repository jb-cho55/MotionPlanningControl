function base_map = generate_map(map_boundary)
%GENERATE_MAP  Build the baseline occupancy grid (drivable area mask).
%
%   base_map = generate_map(map_boundary)
%
%   Input
%       map_boundary : Nx2 or 2xN array of polygon vertices [x y] that bound
%                      the DRIVABLE region of the parking lot.  Cells inside
%                      the polygon are free (0); cells outside are occupied
%                      (1).
%
%   Output
%       base_map : 200x200 uint8 grid (1 = off-road / obstacle, 0 = drivable).
%                  Grid covers x in [X_MIN, X_MAX], y in [Y_MIN, Y_MAX]
%                  at RES m / cell (see map_const.m).
%
%   This is the baseline grid that add_obstacle then ORs the truck footprints
%   on top of.  Keeping these as two stages matches the existing .slx
%   data-flow (MATLAB Function1 -> MATLAB Function2 -> Parking) and lets each
%   module be unit-tested independently.
%
%   The polygon membership test is a ray-cast (even-odd) algorithm operating
%   on a closed polygon — codegen-compatible because the buffer is fixed
%   size (MAX_BOUND = 16 vertices is plenty for the rectangular parking lot).
%
%#codegen

c = map_const();
N = c.N;
res = c.RES;
x_min = c.X_MIN;
y_max = c.Y_MAX;

MAX_BOUND = int32(16);

% Always work on fixed-size buffers (codegen-friendly).  Accepts:
%   - Mx2 [x y] rows
%   - 2xM [x; y] columns
%   - flat (2M)x1 or 1x(2M) interleaved [x1; y1; x2; y2; ...]
%     (this is what .slx's stacked Mux blocks usually produce)
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

% Default to "all occupied" if polygon is degenerate.
if M < 3
    base_map = ones(N, N, 'uint8');
    return;
end

% Sort vertices CCW around the centroid so the polygon is always simple
% regardless of the order the caller (e.g. a .slx Mux block) provides.
% Without this, swapping two vertices yields a self-intersecting
% "bowtie" polygon and point-in-polygon misclassifies half the grid.
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

% Selection sort (codegen-friendly, M is at most MAX_BOUND).
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

for row = 1:N
    wy = y_max - (double(row) - 0.5) * res;
    for col = 1:N
        wx = x_min + (double(col) - 0.5) * res;
        if ~point_in_polygon(wx, wy, bx, by, M)
            base_map(row, col) = uint8(1);
        end
    end
end

end

function inside = point_in_polygon(px, py, bx, by, M)
%#codegen
% Ray-cast (even-odd) test against polygon (bx,by) of length M.
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
