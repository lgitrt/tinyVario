function [vz_est, alt_est, az_bias_est, P_diag, innov] = ...
         fcn_kalman_vz(az_world, alt_baro, baro_new, param, ax, ay, az)
%FCN_KALMAN_VZ  Kalman filter for vertical speed estimation — Simulink block.
%
%  Fuses world-frame vertical acceleration (from Madgwick) with barometer
%  altitude to estimate vertical speed and accelerometer bias.
%
%  STATE VECTOR  x = [h; vz; az_bias]
%    h        : altitude above reference [m]   (NED convention: up positive)
%    vz       : vertical speed, up positive [m/s]
%    az_bias  : vertical accelerometer bias in world frame [m/s2]
%
%  INPUTS
%    az_world   : world-frame vertical specific force from Madgwick [m/s2]
%                 (NED: positive down; we convert internally)
%                 Input convention: az_world > 0 → downward acceleration
%    alt_baro   : barometer altitude measurement [m] (ZOH from 25 Hz)
%    baro_new   : scalar flag, 1 = new baro sample this step, 0 = old
%    param      : parameter bus (see params_variometer.m)
%
%  OUTPUTS
%    vz_est     : estimated vertical speed, up positive [m/s]
%    alt_est    : estimated altitude [m]
%    az_bias_est: estimated accel bias [m/s2]
%    P_diag     : diagonal of covariance P (3×1), for monitoring
%    innov      : barometer innovation [m] (NaN when no new measurement)
%
%  SIMULINK USAGE
%    Block type : MATLAB Function
%    Sample time: param.dt_imu (same as Madgwick block)
%    Feed-through: az_world → no state feedback needed (direct throughput to output)
%
%  ALGORITHM
%    Prediction (every step at IMU rate):
%      x_pred = F*x + B*u       (u = az_up = -az_world_ned - g... see below)
%      P_pred = F*P*F' + Q
%
%    Update (only when baro_new == 1):
%      Innovation:  y    = alt_baro - H*x_pred
%      Gate check:  y² / (H*P_pred*H' + R) < gate_threshold
%      Gain:        K    = P_pred*H' / (H*P_pred*H' + R)
%      State:       x    = x_pred + K*y
%      Covariance:  P    = (I - K*H)*P_pred*(I - K*H)' + K*R*K'  (Joseph form)

%#codegen

persistent x_est;       % state estimate [h; vz; az_bias]
persistent P;           % covariance matrix 3×3

%% Initialise on first call
if isempty(x_est)
    x_est = param.kf.x0(:);           % [h; vz; az_bias]
    x_est(1) = alt_baro;               % initialise altitude from barometer
    P = param.kf.P0;
end

g    = param.simul.g;       % 9.80665 m/s2
F    = param.kf.F;        % 3×3 discrete state transition
B    = param.kf.B;        % 3×1 control input matrix
Q    = param.kf.Q;        % 3×3 process noise covariance
H    = param.kf.H;        % 1×3 measurement matrix [1 0 0]
R    = param.kf.R_h;      % scalar baro noise variance
gate = param.kf.gate_threshold;


% % adaptive Q to reduce weight on accelerometer when entering or being in a
% % turn
% a_mag    = sqrt(ax^2+ay^2+az^2);
% dev      = abs(a_mag - g) / g;
% q_scale  = 1.0 + 15.0 * min(dev, 1.0);   % up to 16x inflation at high bank
Q_adapt  = Q;%*q_scale;

% %adaptive R 
% % Inflate R when vertical acceleration is large, during aggressive thermals the barometer is also contaminated by dynamic pressure changes from airspeed variations
% a_up = -(az_world + g);
R_adapt = R; %* (1.0 + 2.0 * (a_up^2 / g^2));

%% Convert az_world (NED, down positive) to specific force up
% Accelerometer measures: f_body = a - g_body
% After rotation to world (NED): f_ned_down = a_ned_down - g
%   where g ≈ 9.81 m/s2 (positive in NED down direction)
% Vertical acceleration up:
%   a_up = g - f_ned_down = g - az_world   (if az_world is NED down component)
%
% Alternatively, if user provides az_world in body-up convention:
%   a_up = az_world + g - (2*g) = az_world - g  ... depends on sign
%
% ** Convention adopted here: **
%   az_world is the NED-DOWN component of specific force from Madgwick
%   (i.e., rotate_vec_z returns the downward component, positive = down)
%   => a_up_world = -(az_world) - g   (specific force down → negate → accel up, then subtract gravity effect)
%
% More precisely:
%   specific force (NED-down) = f_z_ned
%   a_z_ned (kinematic, down)  = f_z_ned + g
%   a_up                       = -a_z_ned = -(f_z_ned + g)
%
% So: control input u = a_up_corrected = -(az_world + g)
%   (bias is estimated and subtracted by the filter state)
u = -(az_world + g);   % [m/s2] vertical acceleration, up positive
                        % filter state az_bias will estimate residual bias

%% =========================================================
%  PREDICTION STEP
% ==========================================================
% Unbiased control: u_corrected = u - az_bias_est
%  But we fold the bias into the state transition:
%  x_pred = F*x + B*(u - 0)  and F already models bias subtraction via F[1,3]
%  The F matrix has F(1,3)=-dt²/2 and F(2,3)=-dt which subtracts bias effect
u_ctrl = u;   % raw, bias removed implicitly by F*x (x(3) = az_bias)

x_pred = F * x_est + B * u_ctrl;
P_pred = F * P * F' + Q_adapt;

% Symmetrise P
P_pred = 0.5 * (P_pred + P_pred');

%% =========================================================
%  UPDATE STEP (only if new barometer measurement)
% ==========================================================
innov = NaN;

if baro_new >= 0.5   % baro_new flag is 1
    % Innovation
    y   = alt_baro - H * x_pred;
    S   = H * P_pred * H' + R_adapt;     % innovation covariance (scalar)
    
    % Chi-squared innovation gate
    if (y^2 / S) <= gate
        % Kalman gain
        K = P_pred * H' / S;       % 3×1
        
        % State update
        x_pred = x_pred + K * y;
        
        % Covariance update — Joseph form for numerical stability
        I_KH  = eye(3) - K * H;
        P_pred = I_KH * P_pred * I_KH' + K * R_adapt * K';
        P_pred = 0.5 * (P_pred + P_pred');
        
        innov = y;
    end
end

%% Store updated state
x_est = x_pred;
P     = P_pred;

%% Enforce physical constraints
% Altitude cannot go below zero (ground)
if x_est(1) < 0
    x_est(1) = 0;
    if x_est(2) < 0
        x_est(2) = 0;  % can't go negative vz below ground
    end
end

% Limit extreme bias estimates (±2 m/s2 = ~0.2g max accel bias)
x_est(3) = max(-2, min(2, x_est(3)));

%% Outputs
vz_est      = x_est(2);          % [m/s] vertical speed, up positive
alt_est     = x_est(1);          % [m]   altitude
az_bias_est = x_est(3);          % [m/s2] accel bias
P_diag      = diag(P);           % [3×1] covariance diagonal for monitoring
% innov already set above (NaN if no baro update, value if updated)

end  % fcn_kalman_vz