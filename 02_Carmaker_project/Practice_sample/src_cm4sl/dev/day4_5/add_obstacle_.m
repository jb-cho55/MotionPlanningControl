function y = add_obstacle_(map, traffic_info, traffic_size)
%ADD_OBSTACLE_  Self-contained obstacle overlay for the Day4_5_Scenario_1.slx
%   "MATLAB Function2" block.
%
%   y = add_obstacle_(map, traffic_info, traffic_size)
%
%   Inputs
%       map           : 200x200 baseline drivable mask (from generate_map_).
%       traffic_info  : 21x1 — T01..T07 rear-bumper poses [x;y;yaw;...].
%       traffic_size  : 1x2 — Volvo FH truck [W L] = [2.48 11.5].
%
%   Output
%       y             : 200x200 occupancy grid (1 = occupied, 0 = free).
%
%   This file mirrors the chart script verbatim so the .slx is independent
%   of the rest of dev/day4_5/.
%
%#codegen

y = double(add_obstacle_local(uint8(map), traffic_info, traffic_size));
end

%% =====================================================================
%% Local helper functions (inlined from add_obstacle.m + map_const.m)
%% =====================================================================

function occ = add_obstacle_local(base_map, traffic_info, traffic_size)
%#codegen
c = map_const_local();
N = c.N;
res = c.RES;
x_min = c.X_MIN;
y_max = c.Y_MAX;

veh_w = traffic_size(1);
veh_l = traffic_size(2);
margin = c.EGO_W * 0.5 + c.SAFETY_MARGIN;
half_w = veh_w * 0.5 + margin;

occ = base_map;

num_traffic = int32(7);
for i = 1:num_traffic
    idx = (i - 1) * 3;
    rear_x = traffic_info(idx + 1);
    rear_y = traffic_info(idx + 2);
    yaw    = traffic_info(idx + 3);

    if abs(rear_x) < 1.0e-6 && abs(rear_y) < 1.0e-6 && abs(yaw) < 1.0e-6
        continue;
    end

    c_yaw = cos(yaw);
    s_yaw = sin(yaw);

    % Only scan the cells inside the footprint's world AABB instead of the
    % whole 200x200 grid.  The 4 inflated-footprint corners (local
    % x in [-margin, veh_l+margin], y in [-half_w, half_w]) map to world via
    % the forward rotation (dx = lx*c - ly*s, dy = lx*s + ly*c); their
    % min/max give the AABB.  The inner footprint test below is unchanged,
    % so the resulting grid is identical — only the scan region shrinks.
    lxs = [-margin, -margin, veh_l + margin, veh_l + margin];
    lys = [-half_w,  half_w, half_w,        -half_w];
    wx_min =  1.0e18; wx_max = -1.0e18;
    wy_min =  1.0e18; wy_max = -1.0e18;
    for ci = 1:4
        cwx = rear_x + lxs(ci) * c_yaw - lys(ci) * s_yaw;
        cwy = rear_y + lxs(ci) * s_yaw + lys(ci) * c_yaw;
        if cwx < wx_min; wx_min = cwx; end
        if cwx > wx_max; wx_max = cwx; end
        if cwy < wy_min; wy_min = cwy; end
        if cwy > wy_max; wy_max = cwy; end
    end
    % world -> cell index (col grows with x; row grows as y decreases)
    col_min = int32(floor((wx_min - x_min) / res)) + int32(1);
    col_max = int32(floor((wx_max - x_min) / res)) + int32(1);
    row_min = int32(floor((y_max - wy_max) / res)) + int32(1);
    row_max = int32(floor((y_max - wy_min) / res)) + int32(1);
    % +/-1 cell guard against floor rounding, then clamp to [1, N]
    col_min = max(int32(1), col_min - int32(1));
    col_max = min(N,        col_max + int32(1));
    row_min = max(int32(1), row_min - int32(1));
    row_max = min(N,        row_max + int32(1));

    for row = row_min:row_max
        wy = y_max - (double(row) - 0.5) * res;
        for col = col_min:col_max
            wx = x_min + (double(col) - 0.5) * res;
            dx = wx - rear_x;
            dy = wy - rear_y;
            local_x =  dx * c_yaw + dy * s_yaw;
            local_y = -dx * s_yaw + dy * c_yaw;
            if local_x >= -margin && local_x <= veh_l + margin && ...
                    local_y >= -half_w && local_y <= half_w
                occ(row, col) = uint8(1);
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
c.PARK_BOX_W = 3.0;
c.PARK_TOL   = 0.05;
end
