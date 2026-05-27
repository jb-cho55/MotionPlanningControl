function [steer_cmd, dir_sign] = stanley(ego_x, ego_y, ego_yaw, ego_v, ...
                                          path_x, path_y, path_yaw, path_dir, ...
                                          path_len, goal_yaw)
%STANLEY  Lateral controller (Stanley method) for Day4_5 path tracking.
%
%   steer_cmd = stanley(ego_x, ego_y, ego_yaw, ego_v, ...
%                       path_x, path_y, path_yaw, path_len, goal_yaw)
%
%   Inputs
%       ego_x, ego_y, ego_yaw : ego pose at the front-axle reference
%                               (Car.Fr1.tx/ty/rz from CarMaker).
%       ego_v                 : ego longitudinal speed (m/s).
%       path_x, path_y        : fixed-size MAX_PATH buffers.
%       path_yaw              : per-sample heading along the path (rad).
%                               When the planner does not supply per-sample
%                               yaw, pass zeros and the function will
%                               derive it from consecutive (x, y) deltas.
%       path_len              : int32, number of valid samples.
%       goal_yaw              : T00 target heading (used near the end of
%                               the path where path_yaw is undefined or
%                               unreliable).
%
%   Output
%       steer_cmd : front-wheel angle command (rad), saturated.
%
%   Algorithm (textbook Stanley):
%       1. Find the closest path sample to (ego_x, ego_y).
%       2. heading_err = wrap_pi(path_heading - ego_yaw)
%             - path_heading from path_yaw(nearest) when valid,
%               otherwise from atan2(diff y, diff x).
%             - within END_RADIUS of the final path sample, smoothly
%               blend toward goal_yaw so the controller pivots the ego
%               into the parking heading instead of drifting past T00.
%       3. cross_track_err: signed lateral offset of (ego_x, ego_y) from
%          the path-tangent line at the nearest sample (left of heading
%          positive).
%       4. steer = heading_err + atan2(K_E * cross_track_err,
%                                      V_SOFT + |ego_v|)
%       5. Saturate to +/- MAX_STEER.
%
%   Unlike pure pursuit, Stanley drives heading error explicitly to zero,
%   so the ego ends up aligned with the path tangent (= goal_yaw near the
%   end), which is what parking demands.
%
%#codegen

c = map_const();
MAX_PATH    = int32(300);
MAX_STEER   = 0.5;
K_E         = 1.5;          % cross-track gain
V_SOFT      = 1.0;          % low-speed softening (m/s)
END_RADIUS  = 3.0;          % blend to goal_yaw within this many meters

steer_cmd = 0.0;
dir_sign = int8(1);   % +1 forward, -1 reverse
if path_len < int32(2)
    return;
end

plen = int32(min(int32(path_len), MAX_PATH));

% --- 1. Nearest sample (linear scan, cheap at plen <= 300) ---
nearest_idx = int32(1);
best_d2 = 1.0e18;
for i = int32(1):plen
    dx = path_x(i) - ego_x;
    dy = path_y(i) - ego_y;
    d2 = dx*dx + dy*dy;
    if d2 < best_d2
        best_d2 = d2;
        nearest_idx = i;
    end
end

% --- 2. Path heading at the nearest sample ---
% Prefer the supplied path_yaw if it's been populated.
have_path_yaw = false;
if numel(path_yaw) >= double(plen)
    if any(abs(path_yaw(1:min(plen, int32(20)))) > 1.0e-6)
        have_path_yaw = true;
    end
end

if have_path_yaw
    path_heading = path_yaw(nearest_idx);
else
    % Derive from consecutive samples.
    if nearest_idx < plen
        path_heading = atan2(path_y(nearest_idx+1) - path_y(nearest_idx), ...
                             path_x(nearest_idx+1) - path_x(nearest_idx));
    elseif nearest_idx > 1
        path_heading = atan2(path_y(nearest_idx) - path_y(nearest_idx-1), ...
                             path_x(nearest_idx) - path_x(nearest_idx-1));
    else
        path_heading = goal_yaw;
    end
end

% Distance from nearest sample to the path end (along the path index axis).
% Used to blend toward goal_yaw when ego is in the final approach.
end_dist = 0.0;
prev_x = path_x(nearest_idx);
prev_y = path_y(nearest_idx);
for i = nearest_idx+int32(1):plen
    end_dist = end_dist + hypot(path_x(i) - prev_x, path_y(i) - prev_y);
    prev_x = path_x(i);
    prev_y = path_y(i);
end

if end_dist < END_RADIUS
    % Blend factor in [0,1]: 0 at END_RADIUS, 1 at path end.
    alpha = 1.0 - end_dist / END_RADIUS;
    if alpha < 0.0; alpha = 0.0; end
    if alpha > 1.0; alpha = 1.0; end
    diff = wrap_pi(goal_yaw - path_heading);
    path_heading = wrap_pi(path_heading + alpha * diff);
end

% Direction of motion at the nearest path segment (+1 forward, -1 reverse).
% In reverse, the bicycle model flips the relationship between steer and
% turn direction, and the "heading" the ego must align is path_heading + pi
% (ego rear bumper goes forward along path while its yaw points opposite).
if numel(path_dir) >= double(nearest_idx)
    dir_sign = int8(sign_default(path_dir(nearest_idx), int8(1)));
end

eff_heading = path_heading;
if dir_sign == int8(-1)
    eff_heading = wrap_pi(path_heading + pi);
end
heading_err = wrap_pi(eff_heading - ego_yaw);

% --- 3. Cross-track error (signed, Stanley convention) ---
% e_fa > 0 when the PATH is to the LEFT of the vehicle (so a positive
% steer pulls ego back onto the path).  Equivalently, ego on the right
% side of the path tangent -> e_fa > 0.
dx = ego_x - path_x(nearest_idx);
dy = ego_y - path_y(nearest_idx);
c_ph = cos(path_heading);
s_ph = sin(path_heading);
cross_track =  s_ph * dx - c_ph * dy;

% --- 4. Stanley law ---
% In reverse, both terms have to flip sign: the steer-to-yaw kinematic
% gain is inverted because the bicycle pivot is now ahead of the body,
% and the cross-track correction must push the ego front the OTHER way.
cross_track_eff = cross_track;
if dir_sign == int8(-1)
    cross_track_eff = -cross_track;
end
steer_cmd = heading_err + atan2(K_E * cross_track_eff, V_SOFT + abs(ego_v));
if dir_sign == int8(-1)
    steer_cmd = -steer_cmd;
end

% --- 5. Saturate ---
if steer_cmd > MAX_STEER
    steer_cmd = MAX_STEER;
elseif steer_cmd < -MAX_STEER
    steer_cmd = -MAX_STEER;
end

% Suppress unused-input warning for the wheelbase constant if codegen
% strips it later — keeping a reference in case future versions need it.
u_unused = c.WHEELBASE * 0.0;       %#ok<NASGU>

end


function a = wrap_pi(a)
%#codegen
while a > pi;  a = a - 2.0 * pi; end
while a < -pi; a = a + 2.0 * pi; end
end

function s = sign_default(v, fallback)
%#codegen
if v > 0
    s = int8(1);
elseif v < 0
    s = int8(-1);
else
    s = fallback;
end
end
