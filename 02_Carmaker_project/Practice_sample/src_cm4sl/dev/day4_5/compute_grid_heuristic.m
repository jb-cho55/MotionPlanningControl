function h_grid = compute_grid_heuristic(occ_map, gx, gy)
%COMPUTE_GRID_HEURISTIC  Obstacle-aware admissible heuristic for Hybrid A*.
%
%   h_grid = compute_grid_heuristic(occ_map, gx, gy)
%
%   Returns a 200x200 single-precision matrix where h_grid(row,col) is the
%   shortest 8-connected free-space distance (in meters) from the cell to
%   the goal cell.  Occupied cells get a sentinel large value (1e9) so any
%   path that would touch them is dominated.
%
%   This is the obstacle-aware heuristic that turns Hybrid A* from a
%   "greedy Euclidean search" into a complete planner that respects the
%   true geometry of the parking lot — fixes the failure mode where the
%   Euclidean heuristic kept slamming the planner into the L-shaped truck
%   wall.
%
%   Algorithm: standard label-correcting BFS-like Dijkstra on the 8-
%   connected free-cell graph, starting from the goal cell.  Edge weight
%   = sqrt(dr^2 + dc^2) * RES.  Codegen-friendly: bounded queue size,
%   fixed-size buffers, no recursion.
%
%#codegen

c = map_const();
N = double(c.N);
res = c.RES;
N_int = int32(N);

INF = single(1.0e9);

% Output buffer
h_grid = INF * ones(N, N, 'single');

% Goal cell
gc = floor((gx - c.X_MIN) / res) + 1;
gr = floor((c.Y_MAX - gy) / res) + 1;
if gc < 1 || gc > N || gr < 1 || gr > N
    return;  % goal off the grid -> all-INF heuristic (planner falls back)
end
if occ_map(int32(gr), int32(gc)) > 0
    return;  % goal in obstacle -> infeasible, leave all-INF
end

% Linear FIFO queue (label-correcting Bellman-Ford / SPFA style).  Bound
% is loose because each cell can be re-queued at most a few times before
% its label stabilises.  N*N*8 covers the worst case safely.
MAX_Q = int32(8 * N * N);
qr = zeros(MAX_Q, 1, 'int32');
qc = zeros(MAX_Q, 1, 'int32');
qhead = int32(1);
qtail = int32(2);                                 % next write slot

h_grid(int32(gr), int32(gc)) = single(0);
qr(1) = int32(gr);
qc(1) = int32(gc);

DR = int32([-1 -1 -1  0  0  1  1  1]);
DC = int32([-1  0  1 -1  1 -1  0  1]);
SQ2 = single(sqrt(2.0));
COSTS = single([SQ2 1 SQ2 1 1 SQ2 1 SQ2]) * single(res);

while qhead < qtail
    r = qr(qhead);
    cc = qc(qhead);
    qhead = qhead + int32(1);

    h_here = h_grid(r, cc);

    for k = int32(1):int32(8)
        nr = r + DR(k);
        nc = cc + DC(k);
        if nr < 1 || nr > N_int || nc < 1 || nc > N_int
            continue;
        end
        if occ_map(nr, nc) > 0
            continue;
        end
        new_h = h_here + COSTS(k);
        if new_h < h_grid(nr, nc)
            h_grid(nr, nc) = new_h;
            if qtail <= MAX_Q
                qr(qtail) = nr;
                qc(qtail) = nc;
                qtail = qtail + int32(1);
            end
        end
    end
end

end
