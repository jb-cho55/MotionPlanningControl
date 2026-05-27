function desired_ax = pd_speed(v_des, v_ego)
%PD_SPEED  PD longitudinal controller (no integral term).
%
%   desired_ax = pd_speed(v_des, v_ego)
%
%   Inputs
%       v_des : desired longitudinal speed (m/s)
%       v_ego : current ego longitudinal speed (m/s)
%
%   Output
%       desired_ax : commanded longitudinal acceleration (m/s^2),
%                    saturated to [AX_MIN, AX_MAX].
%
%   Form (plan item 3):
%       e   = v_des - v_ego
%       e_d = (e - e_prev) / dt              (1st-order LPF on e_d)
%       desired_ax = Kp * e + Kd * e_d
%       desired_ax = clip(desired_ax, AX_MIN, AX_MAX)
%
%   No integral term to avoid wind-up during long obstacle holds.  The
%   D term passes through a small low-pass (alpha=0.6) so that step
%   changes in v_des don't deliver a derivative spike.
%
%   The fixed time step DT = 0.01 s matches the CarMaker base sample rate
%   (100 Hz).  If used at a different rate, scale e_d accordingly.
%
%#codegen

DT = 0.01;
KP = 1.5;
KD = 0.3;
AX_MIN = -3.0;
AX_MAX = 1.5;
ALPHA  = 0.6;          % LPF coefficient for derivative term

persistent e_prev d_lpf init
if isempty(init)
    e_prev = 0.0;
    d_lpf  = 0.0;
    init   = true;
end

e   = v_des - v_ego;
e_d = (e - e_prev) / DT;
d_lpf = ALPHA * d_lpf + (1.0 - ALPHA) * e_d;
e_prev = e;

desired_ax = KP * e + KD * d_lpf;

if desired_ax > AX_MAX
    desired_ax = AX_MAX;
elseif desired_ax < AX_MIN
    desired_ax = AX_MIN;
end

end
