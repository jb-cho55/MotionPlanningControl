function occ = add_obstacle(base_map, traffic_info, traffic_size)
%ADD_OBSTACLE  Overlay truck footprints onto the baseline occupancy grid.
%
%   occ = add_obstacle(base_map, traffic_info, traffic_size)
%
%   Inputs
%       base_map     : 200x200 uint8 baseline grid from generate_map (1 = off-
%                      road, 0 = drivable).
%       traffic_info : 21x1 double, [x1; y1; yaw1; x2; y2; yaw2; ...; x7; y7; yaw7]
%                      (x, y) is the REAR BUMPER CENTER of truck i in global
%                      coordinates.  yaw is in radians.  An all-zero triple is
%                      treated as a dummy slot and skipped.
%       traffic_size : 1x2 double, [W L]  (Volvo FH = [2.48, 11.5]).
%
%   Output
%       occ : 200x200 uint8 occupancy grid (1 = occupied, 0 = free).
%             Grid covers x in [0,100] m, y in [-100,0] m at 0.5 m / cell.
%             T00 (parking goal) is intentionally NOT drawn — see plan item 5.
%
%   The cell at world (wx, wy) is marked occupied when, in the truck's local
%   frame anchored at the rear bumper center,
%       0 - m <= local_x <= L + m   and   |local_y| <= W/2 + m
%   where m is an inflation that accounts for half the ego width plus a
%   safety margin (see map_const.m).
%
%   This is the from-scratch rewrite called out in plan items 1 and 2 of
%   dev/day4_5/SIGNAL_MAP.md.  Cell count is 200x200 (0.5 m / cell) — 4x the
%   legacy 100x100 / 1 m grid.
%
%#codegen

c = map_const();
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

    for row = 1:N
        wy = y_max - (double(row) - 0.5) * res;
        for col = 1:N
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
