function y = add_obstacle_(map, traffic_info, traffic_size)
%#codegen

y = zeros(200, 200);

for r = 1:200
    for c = 1:200
        y(r, c) = map(r, c);
    end
end

res = 1.0;
x_min = -100.0;
y_max = 100.0;
n = 200;

% Mark off-road cells as obstacles so the visualization clearly shows the
% drivable parking lot. Parking lot (LaneR.0) is x in [0, 100], y in [-100, 0].
% Push the marking 1.5m outside the parking lot so the planner's 0.8m inflation
% in is_occupied_pose does not bleed back into the drivable area at the
% boundary. The hard boundary inside the parking lot is enforced in
% is_occupied_pose's road_x_*/road_y_* check.
for row = 1:n
    wy = y_max - (double(row) - 0.5) * res;
    for col = 1:n
        wx = x_min + (double(col) - 0.5) * res;
        if wx < -1.5 || wx > 101.5 || wy < -101.5 || wy > 1.5
            y(row, col) = 1;
        end
    end
end

veh_w = traffic_size(1);
veh_l = traffic_size(2);

half_w = veh_w * 0.5 + 1.0;
half_l = veh_l * 0.5 + 1.0;

for i = 1:7
    idx = (i - 1) * 3;

    px = traffic_info(idx + 1);
    py = traffic_info(idx + 2);
    yaw = traffic_info(idx + 3);

    if abs(px) < 1e-6 && abs(py) < 1e-6 && abs(yaw) < 1e-6
        continue;
    end

    c_yaw = cos(yaw);
    s_yaw = sin(yaw);

    for row = 1:n
        wy = y_max - (double(row) - 0.5) * res;

        for col = 1:n
            wx = x_min + (double(col) - 0.5) * res;

            dx = wx - px;
            dy = wy - py;

            local_x =  dx * c_yaw + dy * s_yaw;
            local_y = -dx * s_yaw + dy * c_yaw;

            if abs(local_x) <= half_l && abs(local_y) <= half_w
                y(row, col) = 1;
            end
        end
    end
end

end
