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

veh_w = traffic_size(1);
veh_l = traffic_size(2);

% Inflation +1.5m for trucks: the Volvo FH+Silo visual mesh extends past
% the Basics.Dimension=11.5 collision box. Extra margin keeps the path
% clearly away from the visible truck volume but still leaves the corridor
% to the goal at (80, -3) traversable.
half_w = veh_w * 0.5 + 1.5;
half_l = veh_l * 0.5 + 1.5;

% Process the 7 trucks fed in via traffic_info (T01..T07, Volvo FH+Silo)
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

% T00 (IPG_CompanyCar_2018_Blue at (76.1, -5), yaw=pi/2). Hardcoded because the
% Subsystem only routes T01..T07 to traffic_info. Modeled with its actual car
% dimensions (4.47 x 1.97) — using the truck footprint here makes T00 extend
% above y=0 and block the goal approach.
t00_px = 76.1;
t00_py = -5.0;
t00_yaw = pi / 2;
% T00 is a regular car (visual mesh matches collision box well). The
% corridor above T00 to road_y_max=0 is only ~3.2m wide, so use minimal
% padding here — the planner's own 0.8m inflation in is_occupied_pose plus
% the ego half-width still gives ~1.5m clearance from the car body.
t00_half_l = 4.47 * 0.5;
t00_half_w = 1.97 * 0.5;
c_yaw = cos(t00_yaw);
s_yaw = sin(t00_yaw);
for row = 1:n
    wy = y_max - (double(row) - 0.5) * res;
    for col = 1:n
        wx = x_min + (double(col) - 0.5) * res;
        dx = wx - t00_px;
        dy = wy - t00_py;
        local_x =  dx * c_yaw + dy * s_yaw;
        local_y = -dx * s_yaw + dy * c_yaw;
        if abs(local_x) <= t00_half_l && abs(local_y) <= t00_half_w
            y(row, col) = 1;
        end
    end
end

end
