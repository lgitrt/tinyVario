%% validate_filters.m
%
% Author: Luca Obwegs
%
% Standalone, Simulink-free validation of the full sensor-fusion pipeline
% (Madgwick AHRS -> 3-state Kalman filter) against a synthetic paraglider
% trajectory with known ground truth. Calls the exact same reference
% functions that were ported to C for the target (fcn_madgwick.m /
% fcn_kalman_vz.m in Core/Src/matlab_functions), parameterised by the
% development tuning in Core/Src/matlab_functions/param_init.m — the same
% tuning exercised by the Simulink model, so results are directly
% comparable to the Simulink-derived plots without needing Simulink or
% Simscape Multibody installed.
%
% For comparison, a simple barometer-only 2-state Kalman filter (altitude,
% vertical speed; no IMU input) is run side by side as a reference
% baseline, mirroring the "baro-only" comparison used during development.
%
% USAGE
%   >> cd matlab
%   >> validate_filters
%
% OUTPUT
%   Prints RMSE/max-error statistics to the console and saves three plots
%   to ../results/.

clc; close all;
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..', 'Core', 'Src', 'matlab_functions'));

%% =========================================================
%  PARAMETERS — reuse the development tuning checked in at
%  Core/Src/matlab_functions/param_init.m (52 Hz, Simulink-era values).
% ==========================================================
% NOTE: this is the exploratory tuning used while developing and tuning
% the filters in Simulink, not the firmware's shipped production tuning
% (Core/Inc/filter_tuning.h runs the pipeline at 26 Hz with different
% noise constants). Using param_init.m here keeps this script's results
% directly comparable to the Simulink-derived plots, since both exercise
% the same algorithm + the same tuning. Re-tuning the production firmware
% values for the (currently untested) 26 Hz operating point is tracked as
% a follow-up, not done here to avoid presenting unverified numbers.
param_init;   % defines `param`, and (via its final line) also runs
              % trajectory_paraglider to generate ground truth + sensors
              % NOTE: param_init.m starts with `clear`, which wipes `here`
              % above, so paths needed afterwards are recomputed below.
here = fileparts(mfilename('fullpath'));
results_dir = fullfile(here, '..', 'results');
if ~exist(results_dir, 'dir'), mkdir(results_dir); end

dt = param.dt_imu;
N  = numel(t);  %#ok<USENS> -- t, ts_* come from trajectory_paraglider

%% =========================================================
%  RUN THE FILTER CASCADE, SAMPLE BY SAMPLE
% ==========================================================
clear fcn_madgwick fcn_kalman_vz   % reset persistent filter state

az_world_est = zeros(N,1);
alt_est      = zeros(N,1);
vz_est       = zeros(N,1);

% Simple barometer-only reference filter: 2-state [h; vz], constant-
% velocity model, baro-rate updates only (no IMU). Not used on the
% target; included purely as a baseline to put the IMU-aided filter's
% performance into context.
baro_F  = [1, dt; 0, 1];
baro_Q  = [dt^3/3, dt^2/2; dt^2/2, dt] * 0.05;
baro_H  = [1, 0];
baro_x  = [ts_alt_baro.Data(1); 0];
baro_P  = diag([25.0, 4.0]);
alt_est_baro = zeros(N,1);
vz_est_baro  = zeros(N,1);

for k = 1:N
    [~, az_world_est(k)] = fcn_madgwick(ts_ax.Data(k), ts_ay.Data(k), ts_az.Data(k), ...
                                         ts_gx.Data(k), ts_gy.Data(k), ts_gz.Data(k), param);
    [vz_est(k), alt_est(k)] = fcn_kalman_vz(az_world_est(k), ts_alt_baro.Data(k), ...
                                             ts_baro_new.Data(k), param);

    baro_x = baro_F * baro_x;
    baro_P = baro_F * baro_P * baro_F' + baro_Q;
    if ts_baro_new.Data(k) >= 0.5
        y = ts_alt_baro.Data(k) - baro_H * baro_x;
        S = baro_H * baro_P * baro_H' + param.kf.R_h;
        K = baro_P * baro_H' / S;
        baro_x = baro_x + K * y;
        baro_P = (eye(2) - K * baro_H) * baro_P;
    end
    alt_est_baro(k) = baro_x(1);
    vz_est_baro(k)  = baro_x(2);
end

%% =========================================================
%  ERROR METRICS (10 s warm-up excluded, matching firmware ZUPT settle time)
% ==========================================================
ignore_s = round(10 / dt);
rmse   = @(x) sqrt(mean(x(ignore_s:end).^2));
maxabs = @(x) max(abs(x(ignore_s:end)));

az_err         = az_world_est - ts_az_world_true.Data;
altErr_kf      = alt_est      - ts_alt.Data;
vzErr_kf       = vz_est       - ts_vz.Data;
altErr_baro    = alt_est_baro - ts_alt.Data;
vzErr_baro     = vz_est_baro  - ts_vz.Data;

fprintf('\n=== VALIDATION RESULTS (10 s warm-up excluded) ===\n');
fprintf('az_world RMSE                        : %.4f m/s^2\n', rmse(az_err));
fprintf('Altitude RMSE   baro-only KF          : %.4f m\n',     rmse(altErr_baro));
fprintf('Altitude RMSE   3-state KF + Madgwick : %.4f m\n',     rmse(altErr_kf));
fprintf('Vz RMSE         baro-only KF          : %.4f m/s\n',   rmse(vzErr_baro));
fprintf('Vz RMSE         3-state KF + Madgwick : %.4f m/s\n',   rmse(vzErr_kf));
fprintf('Altitude max|err| baro-only KF          : %.4f m\n',   maxabs(altErr_baro));
fprintf('Altitude max|err| 3-state KF + Madgwick : %.4f m\n',   maxabs(altErr_kf));
fprintf('Vz max|err|       baro-only KF          : %.4f m/s\n', maxabs(vzErr_baro));
fprintf('Vz max|err|       3-state KF + Madgwick : %.4f m/s\n', maxabs(vzErr_kf));

%% =========================================================
%  PLOTS
% ==========================================================
% Skip the first second in the plot only (quaternion cold-start
% convergence transient from the identity initial attitude guess);
% RMSE/max-error stats above already exclude a longer 10 s warm-up.
plot_start = round(1 / dt);
fig1 = figure('Name','az_world estimate', 'Color','w', 'Position',[100 100 900 400]);
plot(t(plot_start:end), ts_az_world_true.Data(plot_start:end), 'k', 'LineWidth', 1.2); hold on;
plot(t(plot_start:end), az_world_est(plot_start:end), 'r--', 'LineWidth', 0.9);
grid on; xlabel('Time [s]'); ylabel('a_{z,world} [m/s^2]');
legend('Ground truth','Madgwick estimate','Location','best');
title('Madgwick AHRS — World-Frame Vertical Specific Force');
exportgraphics(fig1, fullfile(results_dir, 'az_world_estimate_standalone.png'), 'Resolution', 150);

fig2 = figure('Name','Altitude error', 'Color','w', 'Position',[100 100 900 400]);
plot(t, altErr_baro, 'Color',[0.6 0.6 0.6], 'LineWidth', 0.9); hold on;
plot(t, altErr_kf, 'b', 'LineWidth', 0.9);
grid on; xlabel('Time [s]'); ylabel('Altitude error [m]');
legend('Baro-only KF','3-state KF + Madgwick','Location','best');
title('Altitude Estimation Error vs. Ground Truth');
exportgraphics(fig2, fullfile(results_dir, 'altitude_error_comparison_standalone.png'), 'Resolution', 150);

fig3 = figure('Name','Vertical speed error', 'Color','w', 'Position',[100 100 900 400]);
plot(t, vzErr_baro, 'Color',[0.6 0.6 0.6], 'LineWidth', 0.9); hold on;
plot(t, vzErr_kf, 'b', 'LineWidth', 0.9);
grid on; xlabel('Time [s]'); ylabel('Vertical speed error [m/s]');
legend('Baro-only KF','3-state KF + Madgwick','Location','best');
title('Vertical Speed Estimation Error vs. Ground Truth');
exportgraphics(fig3, fullfile(results_dir, 'vertical_speed_error_comparison_standalone.png'), 'Resolution', 150);

fprintf('\nSaved plots to %s\n', results_dir);
