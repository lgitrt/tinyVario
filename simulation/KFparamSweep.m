%% =========================================================
%  PART 1: MADGWICK OPTIMIZATION (AHRS)
%  Goal: Minimize error in az_world_est compared to true
% ==========================================================
beta_range = logspace(-4, -0.01, 20); % 0.005 to 0.5
zeta_range = [0, 0.00005, 0.0001, 0.001, 0.003, 0.004, 0.005, 0.006, 0.007, 0.01];

total_ahrs_sims = length(beta_range) * length(zeta_range);
ahrs_results = zeros(total_ahrs_sims, 3); % [beta, zeta, az_rmse]
sim_count = 1;

fprintf('--- PHASE 1: Optimizing Madgwick (%d runs) ---\n', total_ahrs_sims);
t_ahrs = tic;

for b = 1:length(beta_range)
    for z = 1:length(zeta_range)
        
        iter_param = param;
        iter_param.madgwick.beta = beta_range(b);
        iter_param.madgwick.zeta = zeta_range(z);
        
        simIn = Simulink.SimulationInput('EKFsim');
        simIn = simIn.setVariable('param', iter_param);
        
        out = sim(simIn);
        
        % Calculate RMSE against ground truth az_world
        % Skip the first 10 seconds (10 / dt samples) to ignore warm-up
        ignore_samples = round(10 / param.dt_imu);
        az_err = out.azWorldEst.Data(ignore_samples:end) - ts_az_world_true.Data(ignore_samples:end);
        az_rmse = sqrt(mean(az_err.^2));
        
        ahrs_results(sim_count, :) = [beta_range(b), zeta_range(z), az_rmse];
        
        if mod(sim_count, 5) == 0
            fprintf('AHRS Progress: %.1f%% | Best az_RMSE so far: %.4f\n', ...
                (sim_count/total_ahrs_sims)*100, min(ahrs_results(1:sim_count, 3)));
        end
        sim_count = sim_count + 1;
    end
end

% Find best AHRS settings
[~, best_ahrs_idx] = min(ahrs_results(:,3));
best_beta = ahrs_results(best_ahrs_idx, 1);
best_zeta = ahrs_results(best_ahrs_idx, 2);

fprintf('\n--- AHRS OPTIMIZED ---\n');
fprintf('Best Beta: %.4f | Best Zeta: %.4f | az_RMSE: %.4f m/s2\n\n', ...
    best_beta, best_zeta, ahrs_results(best_ahrs_idx, 3));

%% =========================================================
%  PART 2: KALMAN FILTER OPTIMIZATION
%  Goal: Minimize Vz error using the OPTIMIZED AHRS input
% ==========================================================
q_az_range     = logspace(-4, -0.5, 8);  
q_bias_range   = logspace(-3, -2, 8);  
Rh_scale_range = [1.0, 3.0, 5.0, 7.0, 10.0];

total_kf_sims = length(q_az_range) * length(q_bias_range) * length(Rh_scale_range);
kf_results = zeros(total_kf_sims, 5); % [q_az, q_bias, Rh, vz_rmse, alt_rmse]
sim_count = 1;

fprintf('--- PHASE 2: Optimizing Kalman Filter (%d runs) ---\n', total_kf_sims);
t_kf = tic;

for i = 1:length(q_az_range)
    for j = 1:length(q_bias_range)
        for k = 1:length(Rh_scale_range)
            
            iter_param = param;
            % Fix Madgwick to best values found in Phase 1
            iter_param.madgwick.beta = best_beta;
            iter_param.madgwick.zeta = best_zeta;
            
            % Set KF Sweep parameters
            iter_param.kf.q_az = q_az_range(i);
            iter_param.kf.q_bias = q_bias_range(j);
            iter_param.kf.R_h = param.baro.R_baro * Rh_scale_range(k);
            
            % Update Q matrix
            dt = param.dt_imu;
            iter_param.kf.Q = [ iter_param.kf.q_az*dt^3/3,  iter_param.kf.q_az*dt^2/2,  0;
                                iter_param.kf.q_az*dt^2/2,  iter_param.kf.q_az*dt,       0;
                                0,                          0,            iter_param.kf.q_bias*dt ];
            
            simIn = Simulink.SimulationInput('EKFsim');
            simIn = simIn.setVariable('param', iter_param);
            
            out = sim(simIn);
            ignore_samples = round(10 / param.dt_imu);
            vz_rmse = sqrt(mean(out.vzError.Data(ignore_samples:end).^2));
            alt_rmse = sqrt(mean(out.altError.Data(ignore_samples:end).^2));
            
            kf_results(sim_count, :) = [q_az_range(i), q_bias_range(j), Rh_scale_range(k), vz_rmse, alt_rmse];
            
            if mod(sim_count, 20) == 0
                fprintf('KF Progress: %.1f%% | Best Vz_RMSE: %.4f\n', ...
                    (sim_count/total_kf_sims)*100, min(kf_results(1:sim_count, 4)));
            end
            sim_count = sim_count + 1;
        end
    end
end

%% =========================================================
%  FINAL RESULTS & VISUALIZATION
% ==========================================================
[~, best_kf_idx] = min(kf_results(:,4));
fprintf('\n=== GLOBAL SYSTEM OPTIMIZATION COMPLETE ===\n');
fprintf('FINAL AHRS: Beta=%.4f, Zeta=%.4f\n', best_beta, best_zeta);
fprintf('FINAL KF:   q_az=%.2e, q_bias=%.2e, Rh_Scale=%.1f\n', ...
    kf_results(best_kf_idx,1), kf_results(best_kf_idx,2), kf_results(best_kf_idx,3));
fprintf('FINAL RMSE: Vz=%.4f m/s\n', kf_results(best_kf_idx,4));

% Plot AHRS Optimization Map
figure('Name','Madgwick Optimization');
[X, Y] = meshgrid(beta_range, zeta_range);
Z = reshape(ahrs_results(:,3), length(zeta_range), length(beta_range));
contourf(X, Y, Z, 20); set(gca, 'XScale', 'log');
xlabel('Beta'); ylabel('Zeta'); title('AHRS Error (az\_world RMSE)');
colorbar;