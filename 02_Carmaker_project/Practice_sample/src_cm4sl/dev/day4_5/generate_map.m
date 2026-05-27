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

% Normalise map_boundary to Mx2.
if size(map_boundary, 2) == 2
    M = int32(size(map_boundary, 1));
    bx = map_boundary(:, 1);
    by = map_boundary(:, 2);
elseif size(map_boundary, 1) == 2
    M = int32(size(map_boundary, 2));
    bx = map_boundary(1, :).';
    by = map_boundary(2, :).';
else
    M = int32(0);
    bx = zeros(MAX_BOUND, 1);
    by = zeros(MAX_BOUND, 1);
end
if M > MAX_BOUND
    M = MAX_BOUND;
end

% Default to "all occupied" if polygon is degenerate.
if M < 3
    base_map = ones(N, N, 'uint8');
    return;
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
