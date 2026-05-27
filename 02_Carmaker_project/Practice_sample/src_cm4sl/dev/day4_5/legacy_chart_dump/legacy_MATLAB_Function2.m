function y = add_obstacle_(map, traffic_info, traffic_size)
%#codegen
% traffic_info : [rear_x; rear_y; yaw; ...]
% traffic_size : [width, length]
%
% CarMaker traffic position is assumed to be the rear bumper center.
% This function converts rear bumper position to vehicle center position,
% then marks the inflated vehicle footprint as occupied.

y = zeros(100, 100);

for row = 1:100
    for col = 1:100
        y(row, col) = map(row, col);
    end
end

res = 1.0;

x_min = 0.0;
y_max = 0.0;

n_row = 100;
n_col = 100;

veh_w = traffic_size(1);
veh_l = traffic_size(2);

% Inflate obstacle by ego vehicle size.
ego_w = 1.9;
safety_margin = 0.3;

inflation = ego_w * 0.5 + safety_margin;

half_w = veh_w * 0.5 + inflation;
half_l = veh_l * 0.5 + inflation;

num_traffic = 7;

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

    % Convert rear bumper center to vehicle center.
    px = rear_x + veh_l * 0.5 * c_yaw;
    py = rear_y + veh_l * 0.5 * s_yaw;

    for row = 1:n_row
        wy = y_max - (double(row) - 0.5) * res;

        for col = 1:n_col
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