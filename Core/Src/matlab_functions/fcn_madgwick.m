function [q_out, az_world, euler_out] = fcn_madgwick(ax, ay, az, gx, gy, gz, param)
%FCN_MADGWICK  Madgwick AHRS — Fixed for NED (Body Z-Down) convention.
%
%  Author: Luca Obwegs
%
% =========================================================================
%  CONVENTION REFERENCE
% =========================================================================
%  Body frame   : X = forward,  Y = right,  Z = DOWN
%  World frame  : NED — North(X), East(Y), Down(Z)
%  At rest      : accel reads [0, 0, -g] (upward reaction force)
%  az_world     : NED-Down specific force. At rest = -g.
% =========================================================================
%#codegen

persistent q_est;       % [w x y z] row, body→NED
persistent gyro_bias;   % [bx by bz] row, rad/s

if isempty(q_est)
    q_est     = param.madgwick.q0(:)';
    gyro_bias = [0.0, 0.0, 0.0];
end

beta = param.madgwick.beta;   
zeta = param.madgwick.zeta;   
dt   = param.madgwick.dt;     

% Accel-magnitude gate: skip the gradient correction (fall back to pure
% gyro integration) whenever |a| deviates from g by more than
% accel_gate_frac*g — protects attitude estimation during kinematic
% acceleration spikes (turns, turbulence, thermalling). Mirrors
% MADGWICK_ACCEL_GATE_FRAC in Core/Inc/filter_tuning.h and the identical
% gate implemented in Core/Src/madgwick.c. Defaults to the firmware value
% if param.madgwick.accel_gate_frac is not supplied, so existing callers
% (e.g. param_init.m) keep working without modification.
if isfield(param.madgwick, 'accel_gate_frac')
    gate_frac = param.madgwick.accel_gate_frac;
else
    gate_frac = 0.25;
end
g_ref  = param.simul.g;
a_norm = sqrt(ax*ax + ay*ay + az*az);
if a_norm < 1e-6 || a_norm < g_ref*(1-gate_frac) || a_norm > g_ref*(1+gate_frac)
    [q_est, az_world, euler_out] = gyro_only(q_est, ax, ay, az, gx, gy, gz, gyro_bias, dt);
    q_out = q_est;
    return;
end
axn = ax / a_norm;
ayn = ay / a_norm;
azn = az / a_norm;

%% Quaternion components
w = q_est(1); x = q_est(2); y = q_est(3); z = q_est(4);

%% Objective function F (Expected - Measured)
% Gravity in NED is [0, 0, 1]. In Body Frame, the sensor measures 
% acceleration reaction force (-gravity), meaning expected acceleration is -R_bn(3,:)'
F1 = -2.0*(x*z - w*y) - axn;   
F2 = -2.0*(y*z + w*x) - ayn;   
F3 = -(w*w - x*x - y*y + z*z) - azn;

%% Mathematically Correct Jacobian Gradient for Z-Down Gravity (J' * F)
grad_w =  2.0*y*F1 - 2.0*x*F2 - 2.0*w*F3;
grad_x = -2.0*z*F1 - 2.0*w*F2 + 2.0*x*F3;
grad_y =  2.0*w*F1 - 2.0*z*F2 + 2.0*y*F3;
grad_z = -2.0*x*F1 - 2.0*y*F2 - 2.0*z*F3;

grad_norm = sqrt(grad_w^2 + grad_x^2 + grad_y^2 + grad_z^2);
if grad_norm < 1e-10, grad_norm = 1.0; end
grad_w = grad_w / grad_norm;
grad_x = grad_x / grad_norm;
grad_y = grad_y / grad_norm;
grad_z = grad_z / grad_norm;

%% Corrected Gyro bias update (Yaw/Z-observability constraint enforced)
% Gravity cannot observe yaw axis bias drift. Forcing Z-bias correction to 0
% prevents the turning maneuvers from being corrupted as "gyro drift".
gyro_bias(1) = gyro_bias(1) + zeta * grad_x * dt;
gyro_bias(2) = gyro_bias(2) + zeta * grad_y * dt;
gyro_bias(3) = 0.0; % Explicitly locked. No magnetometer = no Z-bias observability.

gx_c = gx - gyro_bias(1);
gy_c = gy - gyro_bias(2);
gz_c = gz - gyro_bias(3);

%% Quaternion derivative (Body-to-NED Integration)
qdot_w = 0.5 * (-x*gx_c - y*gy_c - z*gz_c) - beta * grad_w;
qdot_x = 0.5 * ( w*gx_c + y*gz_c - z*gy_c) - beta * grad_x;
qdot_y = 0.5 * ( w*gy_c - x*gz_c + z*gx_c) - beta * grad_y;
qdot_z = 0.5 * ( w*gz_c + x*gy_c - y*gx_c) - beta * grad_z;

%% Integrate and renormalise
q_est = q_est + [qdot_w, qdot_x, qdot_y, qdot_z] * dt;
q_norm = norm(q_est);
if q_norm < 1e-6, q_est = [1 0 0 0]; else, q_est = q_est / q_norm; end

%% az_world output
% Rotate body specific force [ax, ay, az] to NED frame, take Z component.
w = q_est(1); x = q_est(2); y = q_est(3); z = q_est(4);
R31 = 2.0*(x*z - w*y);    
R32 = 2.0*(y*z + w*x);    
R33 = w*w - x*x - y*y + z*z;
az_world = R31*ax + R32*ay + R33*az;

euler_out = q2euler(q_est);
q_out     = q_est;
end

function [q_new, az_world, euler_out] = gyro_only(q, ax, ay, az, gx, gy, gz, bias, dt)
    % Gyro-only propagation, used while the accelerometer is gated out.
    % az_world is still reported from the (possibly off-1g) instantaneous
    % accel reading rotated through the gyro-propagated attitude — this
    % matches Core/Src/madgwick.c, and is more accurate during real
    % kinematic acceleration than assuming a static 1g reading.
    gx_c = gx - bias(1); gy_c = gy - bias(2); gz_c = gz - bias(3);
    w=q(1); x=q(2); y=q(3); z=q(4);
    qw = w + dt*0.5*(-x*gx_c - y*gy_c - z*gz_c);
    qx = x + dt*0.5*( w*gx_c + y*gz_c - z*gy_c);
    qy = y + dt*0.5*( w*gy_c - x*gz_c + z*gx_c);
    qz = z + dt*0.5*( w*gz_c + x*gy_c - y*gx_c);
    q_new = [qw, qx, qy, qz] / norm([qw, qx, qy, qz]);

    w=q_new(1); x=q_new(2); y=q_new(3); z=q_new(4);
    R31 = 2.0*(x*z - w*y);
    R32 = 2.0*(y*z + w*x);
    R33 = w*w - x*x - y*y + z*z;
    az_world = R31*ax + R32*ay + R33*az;
    euler_out = q2euler(q_new);
end

function euler = q2euler(q)
    w=q(1); x=q(2); y=q(3); z=q(4);
    phi   = atan2(2*(w*x + y*z),  1 - 2*(x*x + y*y));
    sp    = max(-1.0, min(1.0,    2*(w*y - x*z))); 
    theta = asin(sp);
    psi   = atan2(2*(w*z + x*y),  1 - 2*(y*y + z*z));
    euler = [phi, theta, psi];
end