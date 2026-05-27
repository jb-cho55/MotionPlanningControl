function [steer_fl, steer_fr, desired_ax, path_x_dbg, path_y_dbg, path_len_dbg, occ_dbg] = ...
    planner_top(ego_x, ego_y, ego_yaw, ego_v, ...
                t00_x, t00_y, t00_yaw, ...
                traffic_info, traffic_size, map_boundary)
%PLANNER_TOP  Top-level integration for Day4_5 Scenario 1.
%
%   [steer_fl, steer_fr, desired_ax, path_x_dbg, path_y_dbg,
%    path_len_dbg, occ_dbg] =
%       planner_top(ego_x, ego_y, ego_yaw, ego_v,
%                   t00_x, t00_y, t00_yaw,
%                   traffic_info, traffic_size, map_boundary)
%
%   Inputs
%       ego_x, ego_y, ego_yaw, ego_v : ego pose+speed (m, rad, m/s).
%       t00_x, t00_y, t00_yaw        : T00 parking-goal pose (plan item 5).
%       traffic_info                 : 21x1, T01..T07 rear-bumper poses.
%       traffic_size                 : [W L] of the Volvo truck.
%       map_boundary                 : Nx2 drivable polygon vertices.
%
%   Outputs
%       steer_fl, steer_fr : front-wheel angle commands (rad).
%       desired_ax         : longitudinal accel command (m/s^2).
%       path_x/y_dbg       : 300x1 path buffer for monitoring.
%       path_len_dbg       : path length used (int32).
%       occ_dbg            : 200x200 uint8 occupancy grid for monitoring.
%
%   Re-plan policy
%       - First call: always plan.
%       - Goal changed (T00 moved > 0.2 m): re-plan.
%       - Every REPLAN_PERIOD ticks (3 s @ 100 Hz): re-plan.
%       - On plan failure, keep the last valid path (or stay-put fallback).
%
%   v_des profile
%       Distance to goal d:
%           d > 15 m  -> v_des = 3.0 m/s
%           d > 6 m   -> v_des = 1.6 m/s
%           d > 2 m   -> v_des = 0.8 m/s
%           else      -> v_des = 0.0 m/s  (final hold at goal)
%
%#codegen

MAX_PATH = int32(300);
N_OCC = int32(200);
REPLAN_PERIOD = int32(300);   % 3 s @ 100 Hz

persistent path_x path_y path_len occ_cached tick last_gx last_gy init
if isempty(init)
    path_x = zeros(MAX_PATH, 1);
    path_y = zeros(MAX_PATH, 1);
    path_len = int32(0);
    occ_cached = zeros(N_OCC, N_OCC, 'uint8');
    tick = int32(REPLAN_PERIOD + 1);     % force first plan
    last_gx = 1.0e9;
    last_gy = 1.0e9;
    init = true;
end

% --- Decide replan ---
need_replan = false;
if path_len < int32(2)
    need_replan = true;
end
if abs(t00_x - last_gx) > 0.2 || abs(t00_y - last_gy) > 0.2
    need_replan = true;
end
if tick >= REPLAN_PERIOD
    need_replan = true;
end

% --- Plan if needed ---
if need_replan
    base_map = generate_map(map_boundary);
    occ      = add_obstacle(base_map, traffic_info, traffic_size);
    [px, py, ~, plen] = hybrid_astar_plan(ego_x, ego_y, ego_yaw, ...
                                           t00_x, t00_y, t00_yaw, occ);
    if plen >= int32(2)
        % Reshape px/py (which planner returns as 1x300) to 300x1.
        for i = int32(1):MAX_PATH
            path_x(i) = px(i);
            path_y(i) = py(i);
        end
        path_len = plen;
        occ_cached = occ;
        tick = int32(0);
        last_gx = t00_x;
        last_gy = t00_y;
    elseif path_len < int32(2)
        % First-time failure: stay-put 2-point path so the controller
        % has SOMETHING to keep ego at rest.
        path_x(:) = 0.0;
        path_y(:) = 0.0;
        path_x(1) = ego_x;
        path_y(1) = ego_y;
        path_x(2) = ego_x;
        path_y(2) = ego_y;
        path_len = int32(2);
        occ_cached = occ;
        tick = int32(0);
    end
    % Otherwise keep the previous valid path.
end
tick = tick + int32(1);

% --- Speed reference ---
d_goal = hypot(t00_x - ego_x, t00_y - ego_y);
if d_goal > 15.0
    v_des = 3.0;
elseif d_goal > 6.0
    v_des = 1.6;
elseif d_goal > 2.0
    v_des = 0.8;
else
    v_des = 0.0;
end

% --- Controllers ---
steer_cmd  = pure_pursuit(ego_x, ego_y, ego_yaw, ego_v, path_x, path_y, path_len);
desired_ax = pd_speed(v_des, ego_v);

steer_fl = steer_cmd;
steer_fr = steer_cmd;

% --- Debug outputs ---
path_x_dbg   = path_x;
path_y_dbg   = path_y;
path_len_dbg = path_len;
occ_dbg      = occ_cached;

end
