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
%   Prints RMSE/max-error statistics to the console and saves the
%   trajectory overview plus three filter-comparison plots to ../results/.

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

% trajectory_paraglider.m (called above) already produced a diagnostic
% "Paraglider Trajectory Overview" figure (altitude/baro, attitude, IMU
% accel, ground track) — publish it alongside the filter-comparison
% plots below, since it's the clearest single view of the whole
% simulated flight used to generate every other plot in this script.
traj_fig = findobj('Type','figure','Name','Paraglider Trajectory Overview');
if ~isempty(traj_fig)
    exportgraphics(traj_fig(1), fullfile(results_dir, 'trajectory_overview.png'), 'Resolution', 150);
end

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
idx = plot_start:N;

% A 60 s window inside the "tight_thermal1" phase (see trajectory_paraglider.m
% phase table) — the most dynamically demanding segment of the flight
% (45° bank, wing collapses), used as a zoomed-in detail view below the
% full-flight overview so the (very similar) filter traces are easy to
% tell apart instead of fully overlapping at full-flight zoom level.
zoom_t0 = 480; zoom_t1 = 540;

plot_compare(t(idx), ts_az_world_true.Data(idx), 'Ground truth', ...
             az_world_est(idx), 'Madgwick estimate', ...
             'a_{z,world} [m/s^2]', 'Madgwick AHRS — World-Frame Vertical Specific Force', ...
             zoom_t0, zoom_t1, fullfile(results_dir, 'az_world_estimate_standalone.png'));

plot_compare(t, altErr_baro, 'Baro-only KF', ...
             altErr_kf, '3-state KF + Madgwick', ...
             'Altitude error [m]', 'Altitude Estimation Error vs. Ground Truth', ...
             zoom_t0, zoom_t1, fullfile(results_dir, 'altitude_error_comparison_standalone.png'));

plot_compare(t, vzErr_baro, 'Baro-only KF', ...
             vzErr_kf, '3-state KF + Madgwick', ...
             'Vertical speed error [m/s]', 'Vertical Speed Estimation Error vs. Ground Truth', ...
             zoom_t0, zoom_t1, fullfile(results_dir, 'vertical_speed_error_comparison_standalone.png'));

fprintf('\nSaved plots to %s\n', results_dir);

%% =========================================================
%  LOCAL FUNCTIONS
% ==========================================================
function plot_compare(t, sigA, labelA, sigB, labelB, ylab, ttl, zoom_t0, zoom_t1, filename)
%PLOT_COMPARE  Two-panel figure: full-flight overview (top) + zoomed
%detail window (bottom). Both traces are drawn semi-transparent with
%distinct colors/line styles so that near-identical, heavily-overlapping
%lines (as is the case for the baro-only vs. 3-state KF comparisons
%below) both stay visible instead of one fully hiding the other.
colorA = [0.35 0.35 0.35];   % dark gray, solid
colorB = [0.00 0.30 0.85];   % blue, dashed
fig = figure('Color','w', 'Position',[100 100 950 650]);

subplot(2,1,1);
hA = plot(t, sigA, 'Color', colorA, 'LineWidth', 1.4); hA.Color(4) = 0.55; hold on;
hB = plot(t, sigB, '--', 'Color', colorB, 'LineWidth', 1.1); hB.Color(4) = 0.85;
grid on; xlabel('Time [s]'); ylabel(ylab);
legend(labelA, labelB, 'Location','best');
title(ttl);
xline(zoom_t0, ':', 'Color', [0.8 0 0], 'HandleVisibility','off');
xline(zoom_t1, ':', 'Color', [0.8 0 0], 'HandleVisibility','off');

subplot(2,1,2);
zoom_mask = t >= zoom_t0 & t <= zoom_t1;
hA2 = plot(t(zoom_mask), sigA(zoom_mask), 'Color', colorA, 'LineWidth', 1.6); hA2.Color(4) = 0.6; hold on;
hB2 = plot(t(zoom_mask), sigB(zoom_mask), '--', 'Color', colorB, 'LineWidth', 1.3); hB2.Color(4) = 0.9;
grid on; xlabel('Time [s]'); ylabel(ylab);
title(sprintf('Zoomed detail: t = %g-%g s (tight-thermal phase)', zoom_t0, zoom_t1));

exportgraphics(fig, filename, 'Resolution', 150);
end
