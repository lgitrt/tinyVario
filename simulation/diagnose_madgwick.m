%% diagnose_madgwick.m
% Runs fcn_madgwick step-by-step outside Simulink using the ground-truth
% trajectory data, then plots every internal signal so you can pinpoint
% exactly where and why the filter diverges.
%
% Run param_init.m first (which also runs trajectory_paraglider.m).
% All ground-truth signals are taken from the trajectory workspace.
%
% What to look at:
%   1. az_world error  — does it diverge at t~20s? which component of
%                        the quaternion is wrong at that moment?
%   2. Accel norm      — is it >> g during the divergence? (centripetal)
%   3. Gradient magnitude — does it spike or collapse at divergence?
%   4. Gyro bias       — is it drifting in the right direction?
%   5. Euler angle error— which axis (roll/pitch/yaw) goes wrong first?

if ~exist('param','var') || ~exist('t','var')
    param_init;   % loads param + trajectory
end

fprintf('\n=== Madgwick standalone diagnostic ===\n');

%% =========================================================
%  Pull signals from workspace (set by trajectory_paraglider.m)
% ==========================================================
ax_in = ax_meas;   % noisy body-frame accel [m/s²]
ay_in = ay_meas;
az_in = az_meas;
gx_in = gx_meas;   % noisy body-frame gyro [rad/s]
gy_in = gy_meas;
gz_in = gz_meas;

g = param.sim.g;

%% =========================================================
%  PRE-ALLOCATE OUTPUT ARRAYS
% ==========================================================
q_log       = zeros(N, 4);   % quaternion [w x y z]
euler_log   = zeros(N, 3);   % [roll pitch yaw] deg
az_world_log = zeros(N, 1);  % Madgwick az_world output [m/s²]
a_up_log    = zeros(N, 1);   % -(az_world + g) fed to Kalman

% Internal signals for diagnosis
F_log       = zeros(N, 3);   % objective function [F1 F2 F3]
grad_log    = zeros(N, 4);   % normalised gradient [w x y z]
grad_mag_log = zeros(N, 1);  % gradient magnitude before normalisation
accel_norm_log = zeros(N, 1);% ||accel|| — deviation from g reveals manoeuvres
bias_log    = zeros(N, 3);   % gyro bias estimate [rad/s]

%% =========================================================
%  FILTER STATE (mirrors persistent variables in fcn_madgwick)
% ==========================================================
q_est     = param.madgwick.q0(:)';   % [w x y z]
gyro_bias = [0.0, 0.0, 0.0];

beta = param.madgwick.beta;
zeta = param.madgwick.zeta;
dt   = param.madgwick.dt;

%% =========================================================
%  MAIN LOOP — step through every IMU sample
% ==========================================================
for k = 1:N

    ax = ax_in(k);  ay = ay_in(k);  az = az_in(k);
    gx = gx_in(k);  gy = gy_in(k);  gz = gz_in(k);

    % --- Accel norm ---
    a_norm = sqrt(ax^2 + ay^2 + az^2);
    accel_norm_log(k) = a_norm;

    % --- Normalise accel ---
    if a_norm < 1e-6
        % Gyro-only propagation (free-fall guard)
        w=q_est(1); x=q_est(2); y=q_est(3); z=q_est(4);
        gx_c=gx-gyro_bias(1); gy_c=gy-gyro_bias(2); gz_c=gz-gyro_bias(3);
        w_n = w + dt*0.5*(-x*gx_c - y*gy_c - z*gz_c);
        x_n = x + dt*0.5*( w*gx_c + y*gz_c - z*gy_c);
        y_n = y + dt*0.5*( w*gy_c - x*gz_c + z*gx_c);
        z_n = z + dt*0.5*( w*gz_c + x*gy_c - y*gx_c);
        n = sqrt(w_n^2+x_n^2+y_n^2+z_n^2);
        q_est = [w_n,x_n,y_n,z_n]/max(n,1e-6);
        F_log(k,:) = [NaN NaN NaN];
        grad_log(k,:) = [NaN NaN NaN NaN];
        grad_mag_log(k) = NaN;
    else
        axn = ax/a_norm;  ayn = ay/a_norm;  azn = az/a_norm;

        w=q_est(1); x=q_est(2); y=q_est(3); z=q_est(4);

        % --- Objective function ---
        F1 = -2.0*(x*z + w*y) - axn;
        F2 = -2.0*(y*z - w*x) - ayn;
        F3 = -(w*w - x*x - y*y + z*z) - azn;
        F_log(k,:) = [F1, F2, F3];

        % --- Gradient J'*F ---
        gw = -2.0*y*F1 + 2.0*x*F2 - 2.0*w*F3;
        gx_ =  2.0*z*F1 + 2.0*w*F2 + 2.0*x*F3;
        gy_ = -2.0*w*F1 + 2.0*z*F2 + 2.0*y*F3;
        gz_ =  2.0*x*F1 + 2.0*y*F2 - 2.0*z*F3;
        gmag = sqrt(gw^2 + gx_^2 + gy_^2 + gz_^2);
        grad_mag_log(k) = gmag;
        if gmag < 1e-10, gmag = 1.0; end
        gw=gw/gmag; gx_=gx_/gmag; gy_=gy_/gmag; gz_=gz_/gmag;
        grad_log(k,:) = [gw, gx_, gy_, gz_];

        % --- Gyro bias update ---
        gyro_bias(1) = gyro_bias(1) + zeta * gx_ * dt;
        gyro_bias(2) = gyro_bias(2) + zeta * gy_ * dt;
        gyro_bias(3) = gyro_bias(3) + zeta * gz_ * dt;

        gx_c = gx - gyro_bias(1);
        gy_c = gy - gyro_bias(2);
        gz_c = gz - gyro_bias(3);

        % --- Quaternion derivative ---
        qdw = 0.5*(-x*gx_c - y*gy_c - z*gz_c);
        qdx = 0.5*( w*gx_c + y*gz_c - z*gy_c);
        qdy = 0.5*( w*gy_c - x*gz_c + z*gx_c);
        qdz = 0.5*( w*gz_c + x*gy_c - y*gx_c);

        % --- Fuse ---
        qdw = qdw - beta*gw;
        qdx = qdx - beta*gx_;
        qdy = qdy - beta*gy_;
        qdz = qdz - beta*gz_;

        % --- Integrate ---
        w_n = w + qdw*dt;
        x_n = x + qdx*dt;
        y_n = y + qdy*dt;
        z_n = z + qdz*dt;
        qn = sqrt(w_n^2+x_n^2+y_n^2+z_n^2);
        if qn < 1e-6, qn = 1.0; end
        q_est = [w_n, x_n, y_n, z_n] / qn;
    end

    % --- Log bias ---
    bias_log(k,:) = gyro_bias;

    % --- Quaternion & Euler output ---
    q_log(k,:) = q_est;
    w=q_est(1); x=q_est(2); y=q_est(3); z=q_est(4);
    phi_e   = atan2(2*(w*x+y*z), 1-2*(x^2+y^2));
    sp_     = max(-1,min(1, 2*(w*y-z*x)));
    theta_e = asin(sp_);
    psi_e   = atan2(2*(w*z+x*y), 1-2*(y^2+z^2));
    euler_log(k,:) = [phi_e, theta_e, psi_e] * 180/pi;

    % --- az_world ---
    R31 = 2*(x*z + w*y);
    R32 = 2*(y*z - w*x);
    R33 = w^2 - x^2 - y^2 + z^2;
    az_world_log(k) = R31*ax + R32*ay + R33*az;
    a_up_log(k)     = -(az_world_log(k) + g);
end

%% =========================================================
%  GROUND TRUTH SIGNALS (from trajectory workspace)
% ==========================================================
phi_true_deg   = phi   * 180/pi;
theta_true_deg = theta * 180/pi;
psi_true_deg   = psi   * 180/pi;
a_up_true      = -(az_world_true + g);   % Kalman input ground truth

%% =========================================================
%  ERRORS
% ==========================================================
roll_err  = euler_log(:,1) - phi_true_deg;
pitch_err = euler_log(:,2) - theta_true_deg;
yaw_err   = euler_log(:,3) - psi_true_deg;
yaw_err   = mod(yaw_err + 180, 360) - 180;   % wrap to [-180,180]

az_world_err = az_world_log - az_world_true;
a_up_err     = a_up_log - a_up_true;

% Find time of first large divergence
thresh = 1.0;   % [m/s²] error threshold
div_idx = find(abs(az_world_err) > thresh, 1, 'first');
if ~isempty(div_idx)
    fprintf('First divergence > %.1f m/s² at t = %.1f s\n', thresh, t(div_idx));
else
    fprintf('No divergence > %.1f m/s² found — filter looks good!\n', thresh);
end

fprintf('RMS az_world error: %.4f m/s²\n', rms(az_world_err));
fprintf('Max az_world error: %.4f m/s²\n', max(abs(az_world_err)));
fprintf('RMS a_up error:     %.4f m/s²\n', rms(a_up_err));

%% =========================================================
%  FIGURE 1 — PRIMARY DIAGNOSTIC: az_world vs ground truth
% ==========================================================
figure('Name','Madgwick Diagnostic — az_world','NumberTitle','off',...
       'Position',[100 100 1200 800]);

subplot(3,2,1);
plot(t, az_world_true, 'k', 'LineWidth', 1.5); hold on;
plot(t, az_world_log,  'r--', 'LineWidth', 1);
if ~isempty(div_idx)
    xline(t(div_idx), 'b:', 'LineWidth', 1.5);
end
ylabel('[m/s²]'); xlabel('Time [s]');
legend('Ground truth','Madgwick output','Divergence onset');
title('az\_world: ground truth vs Madgwick output');
grid on;

subplot(3,2,2);
plot(t, az_world_err, 'r'); hold on;
yline(0, 'k--');
if ~isempty(div_idx), xline(t(div_idx),'b:'); end
ylabel('[m/s²]'); xlabel('Time [s]');
title('az\_world error (Madgwick − truth)');
grid on;

subplot(3,2,3);
plot(t, a_up_true, 'k', 'LineWidth', 1.5); hold on;
plot(t, a_up_log,  'r--');
if ~isempty(div_idx), xline(t(div_idx),'b:'); end
ylabel('[m/s²]'); xlabel('Time [s]');
legend('True a\_up','Madgwick a\_up');
title('Kalman input a\_up = -(az\_world + g)');
grid on;

subplot(3,2,4);
% Accel norm vs g — when this deviates, accel is NOT measuring pure gravity
% and the gradient correction becomes unreliable
plot(t, accel_norm_log, 'b'); hold on;
yline(g, 'k--', 'LineWidth', 1);
if ~isempty(div_idx), xline(t(div_idx),'b:'); end
ylabel('[m/s²]'); xlabel('Time [s]');
legend('||accel||', 'g = 9.807');
title('Accel norm — deviations from g = non-gravitational acceleration');
grid on;

subplot(3,2,5);
accel_deviation_pct = 100 * abs(accel_norm_log - g) / g;
plot(t, accel_deviation_pct, 'm');
if ~isempty(div_idx), xline(t(div_idx),'b:'); end
yline(5,  'r--', '5%');
yline(10, 'r:',  '10%');
ylabel('[%]'); xlabel('Time [s]');
title('Accel deviation from g [%] — filter unreliable above ~10%');
grid on;

subplot(3,2,6);
plot(t, grad_mag_log, 'g');
if ~isempty(div_idx), xline(t(div_idx),'b:'); end
ylabel('magnitude'); xlabel('Time [s]');
title('Gradient magnitude before normalisation');
grid on;

sgtitle(sprintf('Madgwick Diagnostic  |  beta=%.4f  zeta=%.4f  dt=1/%.0f s',...
        beta, zeta, 1/dt));

%% =========================================================
%  FIGURE 2 — EULER ANGLE ERRORS
% ==========================================================
figure('Name','Madgwick Diagnostic — Euler Angles','NumberTitle','off',...
       'Position',[120 120 1200 700]);

subplot(3,2,1);
plot(t, phi_true_deg, 'k', t, euler_log(:,1), 'r--'); grid on;
legend('True','Madgwick'); ylabel('[°]'); xlabel('Time [s]');
title('Roll \phi'); if ~isempty(div_idx), xline(t(div_idx),'b:'); end

subplot(3,2,2);
plot(t, roll_err, 'r'); grid on; ylabel('[°]'); xlabel('Time [s]');
title('Roll error'); yline(0,'k--'); if ~isempty(div_idx), xline(t(div_idx),'b:'); end

subplot(3,2,3);
plot(t, theta_true_deg, 'k', t, euler_log(:,2), 'r--'); grid on;
legend('True','Madgwick'); ylabel('[°]'); xlabel('Time [s]');
title('Pitch \theta'); if ~isempty(div_idx), xline(t(div_idx),'b:'); end

subplot(3,2,4);
plot(t, pitch_err, 'r'); grid on; ylabel('[°]'); xlabel('Time [s]');
title('Pitch error'); yline(0,'k--'); if ~isempty(div_idx), xline(t(div_idx),'b:'); end

subplot(3,2,5);
plot(t, psi_true_deg, 'k', t, euler_log(:,3), 'r--'); grid on;
legend('True','Madgwick'); ylabel('[°]'); xlabel('Time [s]');
title('Yaw \psi'); if ~isempty(div_idx), xline(t(div_idx),'b:'); end

subplot(3,2,6);
plot(t, yaw_err, 'r'); grid on; ylabel('[°]'); xlabel('Time [s]');
title('Yaw error (wrapped)'); yline(0,'k--'); if ~isempty(div_idx), xline(t(div_idx),'b:'); end

sgtitle('Madgwick — Euler angle errors vs ground truth');

%% =========================================================
%  FIGURE 3 — INTERNAL FILTER SIGNALS
% ==========================================================
figure('Name','Madgwick Diagnostic — Filter Internals','NumberTitle','off',...
       'Position',[140 140 1200 700]);

subplot(3,2,1);
plot(t, F_log(:,1),'b', t, F_log(:,2),'r', t, F_log(:,3),'g'); grid on;
legend('F1','F2','F3'); ylabel('residual'); xlabel('Time [s]');
title('Objective function F — should be ~0 when converged');
if ~isempty(div_idx), xline(t(div_idx),'b:'); end

subplot(3,2,2);
plot(t, grad_log(:,1),'b', t, grad_log(:,2),'r',...
     t, grad_log(:,3),'g', t, grad_log(:,4),'m'); grid on;
legend('grad\_w','grad\_x','grad\_y','grad\_z');
ylabel('normalised'); xlabel('Time [s]');
title('Normalised gradient — large = large attitude error');
if ~isempty(div_idx), xline(t(div_idx),'b:'); end

subplot(3,2,3);
plot(t, bias_log*180/pi); grid on;
legend('bx','by','bz'); ylabel('[°/s]'); xlabel('Time [s]');
title('Gyro bias estimate');
if ~isempty(div_idx), xline(t(div_idx),'b:'); end

subplot(3,2,4);
% Beta effectiveness: compare beta*gradient vs gyro contribution to qdot
% If beta*|grad| >> 0.5*|omega|*dt, accel is dominating — bad during manoeuvres
omega_mag = sqrt(gx_in.^2 + gy_in.^2 + gz_in.^2);
gyro_contrib  = 0.5 * omega_mag * dt;
accel_contrib = beta * ones(N,1);   % beta * |grad_normalised| = beta (always 1 after norm)
plot(t, gyro_contrib, 'b', t, accel_contrib, 'r--'); grid on;
legend('0.5*|\omega|*dt (gyro)', '\beta (accel correction)');
ylabel('[rad/step]'); xlabel('Time [s]');
title('Gyro vs accel contribution per step — accel should be smaller');
if ~isempty(div_idx), xline(t(div_idx),'b:'); end

subplot(3,2,5);
% Quaternion components — should stay smooth; jumps = instability
plot(t, q_log); grid on;
legend('w','x','y','z'); ylabel('value'); xlabel('Time [s]');
title('Quaternion components — smooth = stable');
if ~isempty(div_idx), xline(t(div_idx),'b:'); end

subplot(3,2,6);
% Quaternion norm (should always be 1.0 after renormalisation)
q_norms = sqrt(sum(q_log.^2, 2));
plot(t, q_norms - 1.0, 'k'); grid on;
ylabel('||q|| - 1'); xlabel('Time [s]');
title('Quaternion norm error — should be < 1e-10');
if ~isempty(div_idx), xline(t(div_idx),'b:'); end

sgtitle('Madgwick — Internal filter signals');

%% =========================================================
%  FIGURE 4 — BETA SENSITIVITY ANALYSIS
%  Runs the filter at 5 different beta values so you can
%  pick the one that gives the best az_world tracking.
% ==========================================================
beta_values = [0.01, 0.041, 0.1, 0.2, 0.5];
colors = {'b','g','r','m','k'};

figure('Name','Madgwick Diagnostic — Beta sweep','NumberTitle','off',...
       'Position',[160 160 1200 600]);

subplot(1,2,1); hold on;
plot(t, az_world_true, 'k--', 'LineWidth', 2);
subplot(1,2,2); hold on;

rms_errors = zeros(size(beta_values));

for bi = 1:length(beta_values)
    b_test = beta_values(bi);
    q_b  = param.madgwick.q0(:)';
    gb_b = [0.0, 0.0, 0.0];
    azw_b = zeros(N,1);

    for k = 1:N
        ax=ax_in(k); ay=ay_in(k); az_k=az_in(k);
        gx=gx_in(k); gy=gy_in(k); gz_k=gz_in(k);
        an = sqrt(ax^2+ay^2+az_k^2);
        if an > 1e-6
            axn=ax/an; ayn=ay/an; azn=az_k/an;
            w=q_b(1); x=q_b(2); y=q_b(3); z=q_b(4);
            F1=-2*(x*z+w*y)-axn; F2=-2*(y*z-w*x)-ayn;
            F3=-(w^2-x^2-y^2+z^2)-azn;
            gw=-2*y*F1+2*x*F2-2*w*F3; gx_=-2*w*F1-2*w*F1; % placeholder — full below
            gx_= 2*z*F1+2*w*F2+2*x*F3;
            gy_=-2*w*F1+2*z*F2+2*y*F3;
            gz_= 2*x*F1+2*y*F2-2*z*F3;
            gm=sqrt(gw^2+gx_^2+gy_^2+gz_^2);
            if gm<1e-10, gm=1; end
            gw=gw/gm; gx_=gx_/gm; gy_=gy_/gm; gz_=gz_/gm;
            gb_b(1)=gb_b(1)+zeta*gx_*dt;
            gb_b(2)=gb_b(2)+zeta*gy_*dt;
            gb_b(3)=gb_b(3)+zeta*gz_*dt;
            gx=gx-gb_b(1); gy=gy-gb_b(2); gz_k=gz_k-gb_b(3);
            qdw=0.5*(-x*gx-y*gy-z*gz_k)-b_test*gw;
            qdx=0.5*(w*gx+y*gz_k-z*gy)-b_test*gx_;
            qdy=0.5*(w*gy-x*gz_k+z*gx)-b_test*gy_;
            qdz=0.5*(w*gz_k+x*gy-y*gx)-b_test*gz_;
            w=w+qdw*dt; x=x+qdx*dt; y=y+qdy*dt; z=z+qdz*dt;
            qn=sqrt(w^2+x^2+y^2+z^2); if qn<1e-6, qn=1; end
            q_b=[w,x,y,z]/qn;
        end
        w=q_b(1); x=q_b(2); y=q_b(3); z=q_b(4);
        azw_b(k)=(2*(x*z+w*y))*ax_in(k)+(2*(y*z-w*x))*ay_in(k)+(w^2-x^2-y^2+z^2)*az_in(k);
    end

    err_b = azw_b - az_world_true;
    rms_errors(bi) = rms(err_b);

    subplot(1,2,1);
    plot(t, azw_b, colors{bi}, 'LineWidth', 0.8, ...
         'DisplayName', sprintf('\\beta=%.3f', b_test));

    subplot(1,2,2);
    plot(t, err_b, colors{bi}, 'LineWidth', 0.8, ...
         'DisplayName', sprintf('\\beta=%.3f  RMS=%.3f', b_test, rms_errors(bi)));
end

subplot(1,2,1);
legend('show','Location','best'); grid on;
xlabel('Time [s]'); ylabel('[m/s²]');
title('az\_world for different \beta values');

subplot(1,2,2);
yline(0,'k--'); legend('show','Location','best'); grid on;
xlabel('Time [s]'); ylabel('error [m/s²]');
title('az\_world error for different \beta values');

sgtitle('Beta sensitivity sweep — pick the flattest error curve');

fprintf('\nBeta sweep RMS az_world errors:\n');
for bi = 1:length(beta_values)
    fprintf('  beta = %.3f  =>  RMS = %.4f m/s²\n', beta_values(bi), rms_errors(bi));
end
fprintf('\nOptimal beta from sweep: %.3f\n', beta_values(rms_errors == min(rms_errors)));

fprintf('\nDone. Check figures:\n');
fprintf('  Fig 1: az_world vs ground truth + accel norm\n');
fprintf('  Fig 2: Euler angle errors (which axis diverges first?)\n');
fprintf('  Fig 3: Internal signals (gradient, bias, quaternion)\n');
fprintf('  Fig 4: Beta sweep (what beta value minimises az_world error?)\n');