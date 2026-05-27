function clear_map = compute_clearance(occ_map)
%COMPUTE_CLEARANCE  Distance transform: free-space distance to the nearest
%   occupied cell, in meters.
%
%   clear_map = compute_clearance(occ_map)
%
%   Returns a 200x200 single matrix where clear_map(row, col) is the
%   shortest 8-connected distance from that cell to ANY occupied cell.
%   For occupied cells themselves the value is 0.
%
%   Used by hybrid_astar_plan to shape the search cost so the planner
%   pulls the path toward the centre of the corridor rather than hugging
%   the inflated obstacle boundary.  Same multi-source BFS pattern as
%   compute_grid_heuristic, just with every occupied cell seeded.
%
%#codegen

c = map_const();
N = double(c.N);
res = c.RES;
N_int = int32(N);

INF = single(1.0e9);
clear_map = INF * ones(N, N, 'single');

% Linear FIFO queue.
MAX_Q = int32(8 * N * N);
qr = zeros(MAX_Q, 1, 'int32');
qc = zeros(MAX_Q, 1, 'int32');
qhead = int32(1);
qtail = int32(1);

% Seed: every occupied cell starts at distance 0.
for r = int32(1):N_int
    for cc = int32(1):N_int
        if occ_map(r, cc) > 0
            clear_map(r, cc) = single(0);
            qr(qtail) = r;
            qc(qtail) = cc;
            qtail = qtail + int32(1);
        end
    end
end

DR = int32([-1 -1 -1  0  0  1  1  1]);
DC = int32([-1  0  1 -1  1 -1  0  1]);
SQ2 = single(sqrt(2.0));
COSTS = single([SQ2 1 SQ2 1 1 SQ2 1 SQ2]) * single(res);

while qhead < qtail
    r = qr(qhead);
    cc = qc(qhead);
    qhead = qhead + int32(1);
    d_here = clear_map(r, cc);

    for k = int32(1):int32(8)
        nr = r + DR(k);
        nc = cc + DC(k);
        if nr < 1 || nr > N_int || nc < 1 || nc > N_int
            continue;
        end
        % Skip occupied cells — they're already 0 and already seeded.
        if occ_map(nr, nc) > 0
            continue;
        end
        new_d = d_here + COSTS(k);
        if new_d < clear_map(nr, nc)
            clear_map(nr, nc) = new_d;
            if qtail <= MAX_Q
                qr(qtail) = nr;
                qc(qtail) = nc;
                qtail = qtail + int32(1);
            end
        end
    end
end

end
