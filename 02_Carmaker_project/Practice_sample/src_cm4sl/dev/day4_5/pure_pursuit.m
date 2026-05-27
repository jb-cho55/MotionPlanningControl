function steer_cmd = pure_pursuit(ego_x, ego_y, ego_yaw, ego_v, ...
                                   path_x, path_y, path_len)
%PURE_PURSUIT  Lateral controller for Day4_5 path tracking.
%
%   steer_cmd = pure_pursuit(ego_x, ego_y, ego_yaw, ego_v,
%                            path_x, path_y, path_len)
%
%   Inputs
%       ego_x, ego_y, ego_yaw : ego pose in world frame (m, rad).
%       ego_v                 : ego longitudinal speed (m/s).  Used only
%                               for the velocity-dependent lookahead.
%       path_x, path_y        : fixed-length buffers (MAX_PATH=300) from
%                               hybrid_astar_plan.  Only the first
%                               path_len entries are valid.
%       path_len              : int32, number of valid samples.
%
%   Output
%       steer_cmd : front-wheel steering angle command (rad), saturated
%                   to +/- MAX_STEER.  Output is 0 when path_len < 2.
%
%   Algorithm (plan item 3):
%       1. Find nearest path point to (ego_x, ego_y).
%       2. Walk forward along the path accumulating arc length until
%          Ld = clip(k_v * |v| + Ld_min, Ld_min, Ld_max) is reached.
%       3. Express the resulting target in ego body frame:
%             local_x =  cos(yaw)*dx + sin(yaw)*dy
%             local_y = -sin(yaw)*dx + cos(yaw)*dy
%       4. delta = atan2(2 * L * local_y, Ld^2).
%       5. Saturate.
%
%   When ego is within Ld of the final path point, the target is clamped
%   to that endpoint so the controller still attempts the final heading.
%
%#codegen

c = map_const();
MAX_PATH = int32(300);
L          = c.WHEELBASE;
LD_MIN     = 2.0;
LD_MAX     = 6.0;
K_V        = 0.3;
MAX_STEER  = 0.5;

steer_cmd = 0.0;
if path_len < int32(2)
    return;
end

% --- 1. Nearest point ---
plen = int32(min(int32(path_len), MAX_PATH));
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

% --- 2. Lookahead target ---
Ld = K_V * abs(ego_v) + LD_MIN;
if Ld < LD_MIN
    Ld = LD_MIN;
elseif Ld > LD_MAX
    Ld = LD_MAX;
end

tx = path_x(plen);
ty = path_y(plen);
acc = 0.0;
% Start the accumulation from the nearest point to ego, not from the
% nearest path waypoint.  This avoids missing the lookahead distance
% when ego is already past the nearest sample.
prev_x = ego_x;
prev_y = ego_y;
for i = nearest_idx:plen
    seg = hypot(path_x(i) - prev_x, path_y(i) - prev_y);
    acc = acc + seg;
    if acc >= Ld
        tx = path_x(i);
        ty = path_y(i);
        break;
    end
    prev_x = path_x(i);
    prev_y = path_y(i);
end

% --- 3. Local frame ---
dx = tx - ego_x;
dy = ty - ego_y;
c_yaw = cos(ego_yaw);
s_yaw = sin(ego_yaw);
local_x =  c_yaw*dx + s_yaw*dy;
local_y = -s_yaw*dx + c_yaw*dy;

% Guard against degenerate (target behind ego) — clamp forward distance.
if local_x < 0.1
    local_x = 0.1;
end

% --- 4. Pure pursuit steering ---
Ld_eff = hypot(local_x, local_y);
if Ld_eff < 1.0e-3
    steer_cmd = 0.0;
    return;
end
steer_cmd = atan2(2.0 * L * local_y, Ld_eff * Ld_eff);

% --- 5. Saturate ---
if steer_cmd > MAX_STEER
    steer_cmd = MAX_STEER;
elseif steer_cmd < -MAX_STEER
    steer_cmd = -MAX_STEER;
end

end
