function mapMatrix = generate_map_(map_boundary, traffic_info, traffic_size)
%#codegen
% Generate a 100x100 occupancy map from map boundary and traffic objects.

mapMatrix = zeros(100, 100);

res = 1.0;
x_min = 0.0;
y_max = 0.0;
n = 100;



% map_boundary is expected as [x1; y1; x2; y2; ...].
% The image example builds [0; 0; 0; -100; 100; 0; 100; -100].
% It also accepts an N-by-2 [x y] matrix or a 2-by-N [x; y] matrix.
if size(map_boundary, 2) == 2
    boundary_count = size(map_boundary, 1);
elseif size(map_boundary, 1) == 2
    boundary_count = size(map_boundary, 2);
else
    boundary_count = floor(numel(map_boundary) / 2);
end
x_low =  1.0e9;
x_high = -1.0e9;
y_low =  1.0e9;
y_high = -1.0e9;

for i = 1:boundary_count
    if size(map_boundary, 2) == 2
        x_val = map_boundary(i, 1);
        y_val = map_boundary(i, 2);
    elseif size(map_boundary, 1) == 2
        x_val = map_boundary(1, i);
        y_val = map_boundary(2, i);
    else
        x_val = map_boundary((i - 1) * 2 + 1);
        y_val = map_boundary((i - 1) * 2 + 2);
    end

    if x_val < x_low
        x_low = x_val;
    end
    if x_val > x_high
        x_high = x_val;
    end
    if y_val < y_low
        y_low = y_val;
    end
    if y_val > y_high
        y_high = y_val;
    end
end

% If boundary input is empty or invalid, leave the whole grid available
% except for the outer frame.
if boundary_count >= 2
    margin = 0.0;

    for row = 1:n
        wy = y_max - (double(row) - 0.5) * res;

        for col = 1:n
            wx = x_min + (double(col) - 0.5) * res;

            if wx < x_low - margin || wx > x_high + margin || ...
                    wy < y_low - margin || wy > y_high + margin
                mapMatrix(row, col) = 1;
            end
        end
    end
end

veh_w = traffic_size(1);
veh_l = traffic_size(2);

half_w = veh_w * 0.5;
half_l = veh_l * 0.5;

traffic_count = floor(numel(traffic_info) / 3);

for i = 1:traffic_count
    idx = (i - 1) * 3;

    px = traffic_info(idx + 1);
    py = traffic_info(idx + 2);
    yaw = traffic_info(idx + 3);

    if abs(px) < 1.0e-6 && abs(py) < 1.0e-6 && abs(yaw) < 1.0e-6
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
                mapMatrix(row, col) = 1;
            end
        end
    end
end

end
