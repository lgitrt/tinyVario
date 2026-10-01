%% unit_test_madgwick_trajectory.m
% Validates Madgwick against Paraglider Trajectory with Unwrapped Yaw.
clear; clc;
if ~exist('trajectory_paraglider.mat', 'file')
    error('Trajectory file not found. Run trajectory_paraglider.m first.');
end
load('trajectory_paraglider.mat');

% Simulation Parameters
param.madgwick.beta = 0.1; 
param.madgwick.zeta = 0.0; 
param.madgwick.dt   = dt;

% INITIALIZATION: Sync starting orientation
psi0 = ts_psi.Data(1);
theta0 = ts_theta.Data(1);
phi0 = ts_phi.Data(1);
cp = cos(phi0/2); sp = sin(phi0/2);
ct = cos(theta0/2); st = sin(theta0/2);
cy = cos(psi0/2); sy = sin(psi0/2);
param.madgwick.q0 = [ ...
    cp*ct*cy + sp*st*sy, ...
    sp*ct*cy - cp*st*sy, ...
    cp*st*cy + sp*ct*sy, ...
    cp*ct*sy - sp*st*cy ];

% Pre-allocate
N = length(t);
az_filtered = zeros(N,1);
euler_filtered = zeros(N,3);

fprintf('=== Running Madgwick Trajectory Test ===\n');
for k = 1:N
    [q, az_w, euler] = fcn_madgwick(ts_ax.Data(k), ts_ay.Data(k), ts_az.Data(k), ...
                                    ts_gx.Data(k), ts_gy.Data(k), ts_gz.Data(k), param);
    az_filtered(k) = az_w;
    euler_filtered(k,:) = euler; 
end

%% --- UNWRAP AND ERROR CALCULATION ---
% Wrapping math: atan2(sin(err), cos(err)) finds the shortest distance between angles
phi_err   = rad2deg(atan2(sin(euler_filtered(:,1) - ts_phi.Data), cos(euler_filtered(:,1) - ts_phi.Data)));
theta_err = rad2deg(atan2(sin(euler_filtered(:,2) - ts_theta.Data), cos(euler_filtered(:,2) - ts_theta.Data)));
psi_err   = rad2deg(atan2(sin(euler_filtered(:,3) - ts_psi.Data), cos(euler_filtered(:,3) - ts_psi.Data)));
az_err    = az_filtered - ts_az_world_true.Data;

% Unwrapping for the plot (makes turns look like continuous ramps)
psi_truth_unwrapped = rad2deg(unwrap(ts_psi.Data));
psi_filt_unwrapped  = rad2deg(unwrap(euler_filtered(:,3)));

fprintf('Mean Roll Error:  %.4f deg\n', mean(abs(phi_err)));
fprintf('Mean Pitch Error: %.4f deg\n', mean(abs(theta_err)));
fprintf('Mean Yaw Error:   %.4f deg (Drift over 600s)\n', mean(abs(psi_err)));
fprintf('Mean az_world Error: %.4f m/s²\n', mean(abs(az_err)));

%% --- PLOTTING ---
figure('Name', 'Madgwick Trajectory Validation', 'Color', 'w', 'Position', [100, 100, 800, 900]);

% 1. Roll
subplot(4,1,1);
plot(t, rad2deg(ts_phi.Data), 'k', 'LineWidth', 1.2); hold on;
plot(t, rad2deg(euler_filtered(:,1)), 'r--');
ylabel('Roll [deg]'); grid on; legend('Truth','Madgwick');

% 2. Pitch
subplot(4,1,2);
plot(t, rad2deg(ts_theta.Data), 'k', 'LineWidth', 1.2); hold on;
plot(t, rad2deg(euler_filtered(:,2)), 'b--');
ylabel('Pitch [deg]'); grid on;

% 3. Yaw (Unwrapped to show continuous rotation)
subplot(4,1,3);
plot(t, psi_truth_unwrapped, 'k', 'LineWidth', 1.2); hold on;
plot(t, psi_filt_unwrapped, 'm--');
ylabel('Yaw [deg]'); grid on;
title('Unwrapped Yaw (Total Accumulated Rotation)');

% 4. az_world
subplot(4,1,4);
plot(t, ts_az_world_true.Data, 'k', 'LineWidth', 1.2); hold on;
plot(t, az_filtered, 'g--');
ylabel('az\_world [m/s²]'); xlabel('Time [s]'); grid on;
legend('Truth','Madgwick');

sgtitle('Madgwick Performance on Paraglider Trajectory');