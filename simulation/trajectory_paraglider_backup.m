%% trajectory_paraglider.m
% Generates realistic paraglider flight trajectories as MATLAB timeseries.
% Includes:
%   - Altitude, vertical speed, horizontal speed
%   - Roll, pitch, yaw (Euler angles, ZYX convention, radians)
%   - IMU measurements (accel + gyro, body frame) with sensor noise
%   - Barometer altitude with sensor noise
%   - GROUND TRUTH az_world: NED-Down specific force in world frame [m/s²]
%     This is what the Madgwick filter should output. Use it to diagnose
%     filter errors independently of the Kalman estimator.
%
% Coordinate frame: NED world, body X=forward, Y=right, Z=down
% Euler angles: ZYX → psi (yaw), theta (pitch), phi (roll)
% Quaternion convention: q=[w x y z], body → NED (world)
%
% az_world sign convention (consistent with fcn_madgwick.m):
%   az_world = R_bn(3,:) * f_body   (NED-Down component of specific force)
%   At rest, level: az_world ≈ -g   (specific force opposes gravity)
%   Kalman uses:    a_up = -(az_world + g)  ≈ 0 at rest
%
% EXTENDED TRAJECTORY NOTE:
%   After the original glide2 segment, three aggressive thermal phases are
%   inserted (t = 420–900 s), followed by a weak thermal and approach/landing:
%     • tight_thermal1  : 45° right bank, +4.0 m/s lift
%     • tight_thermal2  : 50° left bank,  +4.8 m/s lift
%     • tight_thermal3  : 55° right bank, +5.0 m/s lift  (strongest/narrowest)
%     • weak_thermal    : 22° left bank,  +0.5 m/s lift  (flat/wide core)
%   Each thermal is followed by a short glide to reset bank and airspeed.
%   Load factors at those bank angles: ~1.41 g / ~1.56 g / ~1.74 g / ~1.08 g
%   Total simulation time: ~1180 s (was 600 s).
%
% Run param_init.m first to load 'param'.

if ~exist('param','var')
    param_init;
end

rng(42);  % reproducible noise

%% =========================================================
%  TIME VECTOR
% ==========================================================
dt   = param.dt_imu;           % simulation step = IMU rate
t    = (0:dt:1180)';           % extended: 600 → 1000 → 1180 s (weak thermal added)
N    = length(t);
g    = param.simul.g;

%% =========================================================
%  FLIGHT PHASE DEFINITIONS
%
%  Original phases (0–420 s) are unchanged.
%  New phases inserted at 420–900 s (three aggressive thermals + glides).
%  Weak thermal inserted at 900–1060 s (low lift, moderate bank).
%  Approach/landing shifted to 1060–1180 s.
%
%  Columns: [t_start, t_end, vz_mean, bank_deg, hdg_rate_dps]
%
%  THERMAL PHYSICS NOTE:
%    • bank 22°: load factor 1/cos(22°) ≈ 1.08 g  (weak/wide thermal)
%    • bank 45°: load factor 1/cos(45°) ≈ 1.41 g
%    • bank 50°: load factor 1/cos(50°) ≈ 1.56 g
%    • bank 55°: load factor 1/cos(55°) ≈ 1.74 g
%    Heading rate for a coordinated turn at airspeed v_air:
%      hdg_rate = g*tan(bank) / v_air  [rad/s]
%    At 22°, v_air=10 m/s: hdg_rate ≈ 9.81*0.404/10 ≈ 4.0 °/s (left)
%    At 45°, v_air=10 m/s: hdg_rate ≈ 9.81*1.0/10   ≈ 9.8 °/s (right)
%    At 50°, v_air=10 m/s: hdg_rate ≈ 9.81*1.19/10  ≈ 11.7 °/s (left)
%    At 55°, v_air=10 m/s: hdg_rate ≈ 9.81*1.43/10  ≈ 14.0 °/s (right)
% ==========================================================
phases = [
%  t_start  t_end   vz_mean  bank_deg  hdg_rate_dps
% ---- ORIGINAL PHASES (unchanged) ----------------------
    0,       15,     3.5,     0,        0;       % launch: climb straight
   15,      120,     2.8,    18,        5;       % thermal1: gentle right circle
  120,      180,    -0.8,     5,       -2;       % glide1: slight left curve
  180,      240,    -0.5,   -22,       -6;       % turn1: left 360°
  240,      360,     3.2,    20,        5;       % thermal2: moderate right circle
  360,      420,    -1.0,     0,        0;       % glide2: straight
% ---- NEW: AGGRESSIVE THERMAL PHASES -------------------
% Transition glide — bleed off bank, reset heading
  420,      450,    -0.5,     0,        0;       % inter-thermal glide
% Tight thermal 1: 45° right bank, strong +4.0 m/s lift
%   hdg_rate ≈ +9.8 °/s → ~3.5 full circles in 120 s
  450,      570,     4.0,    45,        9.8;     % tight_thermal1: 45° right
% Short exit glide
  570,      600,    -0.8,     5,        1;       % exit glide 1
% Tight thermal 2: 50° left bank, very strong +4.8 m/s lift
%   hdg_rate ≈ -11.7 °/s → ~4 full circles in 120 s
  600,      720,     4.8,   -50,      -11.7;     % tight_thermal2: 50° left
% Short exit glide
  720,      750,    -0.8,    -5,       -1;       % exit glide 2
% Tight thermal 3: 55° right bank, maximum +5.0 m/s lift (narrow core)
%   hdg_rate ≈ +14.0 °/s → ~4.7 full circles in 120 s
  750,      870,     5.0,    55,       14.0;     % tight_thermal3: 55° right
% Glide out and lose altitude for approach
  870,      900,    -1.5,     0,        0;       % exit glide 3 / descend
% ---- NEW: WEAK / LOW-LIFT THERMAL ---------------------------------
% Transition into weak thermal: gentle left entry
  900,      920,    -0.2,   -10,       -2;       % weak thermal entry glide
% Weak thermal core: 22° left bank, +0.5 m/s lift
%   hdg_rate ≈ -4.0 °/s → ~5 full circles in 450 s (wide/flat core)
%   load factor ≈ 1.08 g — barely perceptible, typical of a blue thermal
  920,     1040,     0.5,   -22,       -4.0;     % weak_thermal: 22° left
% Exit weak thermal
 1040,     1060,    -0.5,    -5,       -1;       % weak thermal exit
% ---- APPROACH & LANDING (shifted to 1060–1180) --------------------
 1060,     1100,    -0.8,   -15,       -4;       % turn2: left approach turn
 1100,     1150,    -1.5,     5,        1;       % approach: slow descent
 1150,     1180,    -2.0,     0,        0;       % landing flare
];

%% =========================================================
%  SMOOTH TRAJECTORY GENERATION
% ==========================================================
vz_world  = zeros(N,1);   % [m/s] up positive
alt       = zeros(N,1);   % [m]   altitude above start
alt(1)    = 800;

phi   = zeros(N,1);   % roll
theta = zeros(N,1);   % pitch
psi   = zeros(N,1);   % yaw/heading

vN    = zeros(N,1);
vE    = zeros(N,1);
v_air = 10;   % [m/s] nominal airspeed

wind_N = 3.0;
wind_E = 1.5;

psi(1)   = 0;
phi(1)   = 0;
theta(1) = 2 * pi/180;

% Time constants for bank/Vz transitions.
% A tighter tau_phi makes bank transitions snappier (more realistic for
% an experienced pilot entering a thermal core aggressively).
tau_vz_default  = 5.0;
tau_phi_default = 2.0;
tau_phi_tight   = 1.5;   % faster bank transitions in aggressive thermals

for k = 2:N
    tk = t(k);

    ph_idx = find(phases(:,1) <= tk & phases(:,2) > tk, 1, 'last');
    if isempty(ph_idx), ph_idx = size(phases,1); end

    vz_target   = phases(ph_idx, 3);
    bank_target = phases(ph_idx, 4) * pi/180;
    hdg_rate    = phases(ph_idx, 5) * pi/180;

    % Use faster bank settling during aggressive thermal phases (col 4 abs > 40°)
    if abs(phases(ph_idx, 4)) >= 45
        tau_phi = tau_phi_tight;
    else
        tau_phi = tau_phi_default;
    end
    tau_vz = tau_vz_default;

    vz_world(k) = vz_world(k-1) + dt/tau_vz  * (vz_target  - vz_world(k-1));
    phi(k)      = phi(k-1)      + dt/tau_phi * (bank_target - phi(k-1));

    sigma_turb = 0.4;
    tau_turb   = 3.0;
    vz_world(k) = vz_world(k) + sigma_turb * sqrt(2*dt/tau_turb) * randn();

    alt(k) = alt(k-1) + dt * vz_world(k);
    alt(k) = max(alt(k), 0);

    psi(k) = psi(k-1) + dt * hdg_rate;

    % Pitch model: in a banked turn the nose rises slightly relative to the
    % turn axis; add a load-factor-proportional pitch component so the IMU
    % az signal reflects realistic centripetal loading.
    theta_turn = abs(phi(k)) * 0.15;
    theta_vz   = asin(max(-0.3, min(0.3, vz_world(k) / v_air)));
    theta(k)   = 0.7*theta(k-1) + 0.3*(theta_vz + theta_turn);

    vN(k) = v_air * cos(psi(k)) * cos(theta(k)) + wind_N;
    vE(k) = v_air * sin(psi(k)) * cos(theta(k)) + wind_E;
end

%% =========================================================
%  ANGULAR RATES (body frame)
% ==========================================================
dphi   = [0; diff(phi)]   / dt;
dtheta = [0; diff(theta)] / dt;
dpsi   = [0; diff(psi)]   / dt;

p = zeros(N,1);  q_r = zeros(N,1);  r = zeros(N,1);
for k = 1:N
    cp = cos(phi(k)); sp = sin(phi(k));
    ct = cos(theta(k)); tt = tan(theta(k));
    T  = [1,  sp*tt,  cp*tt;
          0,  cp,    -sp;
          0,  sp/ct,  cp/ct];
    omega_body = T * [dphi(k); dtheta(k); dpsi(k)];
    p(k)   = omega_body(1);
    q_r(k) = omega_body(2);
    r(k)   = omega_body(3);
end

%% =========================================================
%  SPECIFIC FORCE IN BODY FRAME
% ==========================================================
ax_body = zeros(N,1);
ay_body = zeros(N,1);
az_body = zeros(N,1);

dvN = [0; diff(vN)] / dt;
dvE = [0; diff(vE)] / dt;
dvz = [0; diff(vz_world)] / dt;
a_ned = [dvN, dvE, -dvz];   % [N E D]

g_ned = [0, 0, g];

%% =========================================================
%  GROUND TRUTH az_world
%  az_world = R_bn(3,:) * f_body  =  NED-Down component of specific force
%             (same formula as fcn_madgwick.m rotate output)
%
%  R_bn row 3 from ZYX Euler angles:
%    R_bn(3,1) = -sin(theta)
%    R_bn(3,2) =  cos(theta)*sin(phi)
%    R_bn(3,3) =  cos(theta)*cos(phi)
%
%  Equivalently via the body-frame specific force directly:
%    f_ned     = a_ned - g_ned          [NED specific force, row vector]
%    f_body    = R_bn' * f_ned'         [body specific force]
%    az_world  = R_bn(3,:) * f_body     [= f_ned(3) = NED-Down component]
%
%  Note: rotating f_body back to NED and taking the Down component
%  is identical to just taking f_ned(3) directly, since R_bn is orthogonal:
%    R_bn * R_bn' = I  =>  R_bn(3,:) * f_body = f_ned(3)
%
%  So the ground truth is simply:
%    az_world_true(k) = a_ned(k,3) - g_ned(3)
%                     = (-dvz(k)) - g
%
%  This is exact, requires no rotation, and is the cleanest reference.
%  We also compute it the long way (via R_bn and f_body) as a cross-check
%  and to provide the body-frame signals for the timeseries.
% ==========================================================
az_world_true = zeros(N,1);   % ground truth NED-Down specific force [m/s²]

for k = 1:N
    cp = cos(phi(k));   sp = sin(phi(k));
    ct = cos(theta(k)); st = sin(theta(k));
    cy = cos(psi(k));   sy = sin(psi(k));

    % R_bn: body → NED  (ZYX: Rz*Ry*Rx)
    R_bn = [cy*ct,  cy*st*sp - sy*cp,  cy*st*cp + sy*sp;
            sy*ct,  sy*st*sp + cy*cp,  sy*st*cp - cy*sp;
           -st,     ct*sp,              ct*cp           ];

    % Specific force in NED (row vector)
    f_ned = a_ned(k,:) - g_ned;

    % Rotate to body frame
    f_body_vec = R_bn' * f_ned';
    ax_body(k) = f_body_vec(1);
    ay_body(k) = f_body_vec(2);
    az_body(k) = f_body_vec(3);

    % Ground truth az_world: rotate f_body back to NED, take Down (Z) component
    % R_bn(3,:) * f_body  =  f_ned(3)  (by orthogonality, exact)
    %
    % Both methods below must agree to numerical precision:
    %   Method A (direct):  f_ned(3)
    %   Method B (via R_bn row 3 from Euler):  -st*ax_body + ct*sp*ay_body + ct*cp*az_body
    %
    % We store Method A as it is exact and free of rotation errors.
    % Method B is written out explicitly so you can verify the R_bn row-3
    % formula used in fcn_madgwick against it.
    az_world_true(k) = f_ned(3);   % Method A — exact ground truth

    % Method B (cross-check, should equal Method A to floating-point precision):
    % az_world_true_B = -st*ax_body(k) + ct*sp*ay_body(k) + ct*cp*az_body(k);
end

% Sanity check: Methods A and B must agree
az_world_true_B = -sin(theta).*ax_body + cos(theta).*sin(phi).*ay_body + cos(theta).*cos(phi).*az_body;
max_AB_error = max(abs(az_world_true - az_world_true_B));
fprintf('az_world ground truth cross-check (A vs B): max error = %.2e m/s²  (should be < 1e-10)\n', max_AB_error);

%% =========================================================
%  ADD VIBRATION AND REALISTIC NOISE
% ==========================================================
% 1. MECHANICAL VIBRATION (Oscillatory noise from lines/wind)
% This creates "spikes" that confuse high Beta values.
f_vib = 15; % 15Hz vibration frequency
vibration_amp = 0.2; % m/s^2 amplitude
vibration = vibration_amp * sin(2 * pi * f_vib * t) .* randn(N,1);

% 2. TURBULENCE (Low-frequency air movement)
% This was previously removed; adding a small amount back 
% makes the Vz RMSE more challenging to optimize.
sigma_turb = 0.15;
tau_turb   = 2.0;
turbulence = zeros(N,1);
for k = 2:N
    turbulence(k) = turbulence(k-1) + (-1/tau_turb * turbulence(k-1)) * dt + ...
                    sigma_turb * sqrt(2*dt/tau_turb) * randn();
end

% 3. UPDATING MEASUREMENTS
sa = param.imu.sigma_a;
sg = param.imu.sigma_g;

% Bias (Random Walk)
ba = cumsum(param.imu.bias_instability_a * sqrt(dt) * randn(N,3), 1);
bg = cumsum(param.imu.bias_instability_g * sqrt(dt) * randn(N,3), 1);

% Final Accel Measurements (Body Frame)
% We add Vibration to the body axes
ax_meas = ax_body + ba(:,1) + sa*randn(N,1) + vibration;
ay_meas = ay_body + ba(:,2) + sa*randn(N,1) + vibration;
az_meas = az_body + ba(:,3) + sa*randn(N,1) + vibration;

% Final Gyro Measurements (Body Frame)
gx_meas = p   + bg(:,1) + sg*randn(N,1);
gy_meas = q_r + bg(:,2) + sg*randn(N,1);
gz_meas = r   + bg(:,3) + sg*randn(N,1);

%% =========================================================
%  BAROMETER
% ==========================================================
t_baro   = (0:param.dt_baro:t(end))';
N_baro   = length(t_baro);

R_air    = 287.058;
exponent = g / (R_air * param.baro.L);

alt_baro_true = interp1(t, alt, t_baro, 'linear', 'extrap');
p_baro_true   = param.baro.p0 * (1 - param.baro.L * alt_baro_true / param.baro.T0).^exponent;
p_baro_meas   = p_baro_true + param.baro.sigma_p * randn(N_baro,1);
alt_baro_meas = (param.baro.T0 / param.baro.L) .* ...
                (1 - (p_baro_meas / param.baro.p0).^(1/exponent));

alt_baro_imu = interp1(t_baro, alt_baro_meas, t, 'previous', 'extrap');

baro_new = zeros(N,1);
[~, baro_idx] = unique(round(t / param.dt_baro));
baro_new(baro_idx) = 1;

%% =========================================================
%  PACK INTO TIMESERIES
% ==========================================================
% Wrap angles to [-pi, pi] for the output timeseries ONLY
psi_wrapped   = mod(psi + pi, 2*pi) - pi;
phi_wrapped   = mod(phi + pi, 2*pi) - pi;
theta_wrapped = mod(theta + pi, 2*pi) - pi;

% True states (Wrapped for standard AHRS comparison)
ts_alt       = timeseries(alt,      t, 'Name','alt_true_m');
ts_vz        = timeseries(vz_world, t, 'Name','vz_true_mps');
ts_phi_wr       = timeseries(phi_wrapped,   t, 'Name','phi_roll_rad_wr');
ts_theta_wr     = timeseries(theta_wrapped, t, 'Name','theta_pitch_rad_wr');
ts_psi_wr       = timeseries(psi_wrapped,   t, 'Name','psi_yaw_rad_wr');
ts_phi       = timeseries(phi,   t, 'Name','phi_roll_rad');
ts_theta     = timeseries(theta, t, 'Name','theta_pitch_rad');
ts_psi       = timeseries(psi,   t, 'Name','psi_yaw_rad');

% Body rates (true)
ts_p         = timeseries(p,   t, 'Name','p_rollrate_radps');
ts_q         = timeseries(q_r, t, 'Name','q_pitchrate_radps');
ts_r         = timeseries(r,   t, 'Name','r_yawrate_radps');

% Position (NED integrated)
posN = cumsum(vN) * dt;
posE = cumsum(vE) * dt;
posD = alt; % NED Down is negative Altitude
ts_px        = timeseries(posN, t, 'Name','px_body_meas_m');
ts_py        = timeseries(posE, t, 'Name','py_body_meas_m');
ts_pz        = timeseries(posD, t, 'Name','pz_body_meas_m');

% IMU measurements (noisy)
ts_ax        = timeseries(ax_meas, t, 'Name','ax_body_meas_mps2');
ts_ay        = timeseries(ay_meas, t, 'Name','ay_body_meas_mps2');
ts_az        = timeseries(az_meas, t, 'Name','az_body_meas_mps2');
ts_gx        = timeseries(gx_meas, t, 'Name','gx_body_meas_radps');
ts_gy        = timeseries(gy_meas, t, 'Name','gy_body_meas_radps');
ts_gz        = timeseries(gz_meas, t, 'Name','gz_body_meas_radps');

% Barometer
ts_alt_baro  = timeseries(alt_baro_imu, t, 'Name','alt_baro_meas_m');
ts_baro_new  = timeseries(baro_new,     t, 'Name','baro_new_flag');

% -----------------------------------------------------------------------
% GROUND TRUTH az_world  [m/s²]
%   NED-Down component of specific force in world frame.
%   Convention: ≈ -g at rest (level flight, no vertical acceleration).
%   Madgwick output should track this signal.
%   Kalman input: a_up = -(az_world + g)
% -----------------------------------------------------------------------
ts_az_world_true = timeseries(az_world_true, t, 'Name','az_world_true_mps2');

% Also save body-frame true specific force (before noise) for reference
ts_ax_body_true = timeseries(ax_body, t, 'Name','ax_body_true_mps2');
ts_ay_body_true = timeseries(ay_body, t, 'Name','ay_body_true_mps2');
ts_az_body_true = timeseries(az_body, t, 'Name','az_body_true_mps2');

%% =========================================================
%  REPORT PHASE STATISTICS
% ==========================================================
fprintf('\n--- Aggressive Thermal Phase Statistics ---\n');
for ph = [6 7 8 9 10 11]   % new phases (1-indexed rows in phases matrix)
    if ph > size(phases,1), break; end
    t0 = phases(ph,1);  t1 = phases(ph,2);
    mask = t >= t0 & t < t1;
    if ~any(mask), continue; end
    lf = 1./cos(phi(mask));   % load factor (g)
    fprintf('Phase %d [%3.0f–%3.0f s]: bank=%.0f° | vz=[%.1f,%.1f] m/s | LF=[%.2f,%.2f] g\n', ...
        ph, t0, t1, phases(ph,4), min(vz_world(mask)), max(vz_world(mask)), min(lf), max(lf));
end
fprintf('\n--- Weak Thermal Phase Statistics ---\n');
wt_mask = t >= 920 & t < 1040;
lf_wt = 1./cos(phi(wt_mask));
fprintf('weak_thermal [920–1040 s]: bank=%.0f° | vz=[%.2f,%.2f] m/s | LF=[%.2f,%.2f] g\n', ...
    -22, min(vz_world(wt_mask)), max(vz_world(wt_mask)), min(lf_wt), max(lf_wt));
fprintf('Peak |az_body| during aggressive thermals: %.2f m/s²\n', ...
    max(abs(az_body(t>=450 & t<870))));
fprintf('Peak load factor (approx):                 %.2f g\n', ...
    max(1./cos(abs(phi(t>=450 & t<870)))));
fprintf('Altitude at start of aggressive section:   %.0f m\n', alt(find(t>=450,1)));
fprintf('Altitude at end   of aggressive section:   %.0f m\n', alt(find(t>=870,1)));
fprintf('Altitude at start of weak thermal:         %.0f m\n', alt(find(t>=920,1)));
fprintf('Altitude at end   of weak thermal:         %.0f m\n', alt(find(t>=1040,1)));
fprintf('Total simulation time:                     %.0f s\n', t(end));

%% =========================================================
%  DIAGNOSTIC PLOTS
% ==========================================================
figure('Name','Paraglider Trajectory (Extended)','NumberTitle','off','Position',[50 50 1400 800]);

subplot(3,3,1);
plot(t, alt); grid on;
xline(450,'r--','TH1'); xline(600,'b--','TH2'); xline(750,'g--','TH3'); xline(920,'m--','TH4');
xlabel('Time [s]'); ylabel('Altitude [m]');
title('Altitude');

subplot(3,3,2);
plot(t, vz_world); grid on;
xline(450,'r--'); xline(600,'b--'); xline(750,'g--'); xline(920,'m--');
yline(0.5,'m:','0.5 m/s'); yline(4.0,'r:','4 m/s'); yline(5.0,'g:','5 m/s');
xlabel('Time [s]'); ylabel('Vz [m/s]');
title('Vertical Speed (up+)');

subplot(3,3,3);
plot(t, phi*180/pi, 'b', t, theta*180/pi, 'r', t, psi*180/pi, 'g');
yline(22,'m:','22°'); yline(-22,'m:');
yline(45,'b:','45°'); yline(-45,'b:'); yline(55,'b--','55°'); yline(-55,'b--');
legend('\phi roll','\theta pitch','\psi yaw'); grid on;
xlabel('Time [s]'); ylabel('Angle [°]');
title('Euler Angles');

subplot(3,3,4);
lf = 1./cos(phi);  lf(abs(phi) > 85*pi/180) = NaN;
plot(t, lf, 'k', 'LineWidth', 1.2); grid on;
yline(1.08,'m--','1.08 g (22°)');
yline(1.41,'r--','1.41 g (45°)'); yline(1.56,'b--','1.56 g (50°)'); yline(1.74,'g--','1.74 g (55°)');
xlabel('Time [s]'); ylabel('Load factor [g]');
title('Instantaneous Load Factor (1/cos\phi)');

subplot(3,3,5);
plot(t, ax_meas, 'b', t, ay_meas, 'r', t, az_meas, 'g');
legend('ax','ay','az'); grid on;
xlabel('Time [s]'); ylabel('[m/s²]');
title('Accel Measurements (body, noisy)');

subplot(3,3,6);
plot(t, gx_meas*180/pi, 'b', t, gy_meas*180/pi, 'r', t, gz_meas*180/pi, 'g');
legend('gx','gy','gz'); grid on;
xlabel('Time [s]'); ylabel('[°/s]');
title('Gyro Measurements (body, noisy)');

subplot(3,3,7);
plot(t, alt, 'k', t, alt_baro_imu, 'b--');
legend('True','Baro meas'); grid on;
xlabel('Time [s]'); ylabel('[m]');
title('Altitude: True vs Baro');

subplot(3,3,8);
a_up_true = -(az_world_true + g);
plot(t, az_world_true, 'k', 'LineWidth', 1.5); hold on;
plot(t, a_up_true, 'r--', 'LineWidth', 1);
yline(-g, 'b:', 'LineWidth', 1);
legend('az\_world\_true [m/s²]', 'a\_up = -(az\_world+g)', '-g reference');
grid on;
xlabel('Time [s]'); ylabel('[m/s²]');
title('az\_world Ground Truth');

subplot(3,3,9);
plot(cumsum(vE)*dt, cumsum(vN)*dt); grid on;
axis equal;
t1_mask = t >= 450  & t < 570;
t2_mask = t >= 600  & t < 720;
t3_mask = t >= 750  & t < 870;
t4_mask = t >= 920  & t < 1040;
hold on;
plot(cumsum(vE(t1_mask))*dt + posE(find(t1_mask,1)), ...
     cumsum(vN(t1_mask))*dt + posN(find(t1_mask,1)), 'r-', 'LineWidth', 2);
plot(cumsum(vE(t2_mask))*dt + posE(find(t2_mask,1)), ...
     cumsum(vN(t2_mask))*dt + posN(find(t2_mask,1)), 'b-', 'LineWidth', 2);
plot(cumsum(vE(t3_mask))*dt + posE(find(t3_mask,1)), ...
     cumsum(vN(t3_mask))*dt + posN(find(t3_mask,1)), 'g-', 'LineWidth', 2);
plot(cumsum(vE(t4_mask))*dt + posE(find(t4_mask,1)), ...
     cumsum(vN(t4_mask))*dt + posN(find(t4_mask,1)), 'm-', 'LineWidth', 2);
legend('Full track','TH1 45°','TH2 50°','TH3 55°','TH4 22° weak','Location','best');
xlabel('East [m]'); ylabel('North [m]');
title('Ground Track (colored = thermals)');

sgtitle('Paraglider Simulation — Extended: Aggressive + Weak Thermals (45°–55° / 22° bank)');

%% =========================================================
%  EXTRACT GROUND TRUTH VERTICAL BIAS
% ==========================================================
% The KF estimates the bias in the world vertical frame.
% Because the bias ba is generated in the body frame, we must 
% project it to the NED Down axis.
az_bias_true = zeros(N,1);

for k = 1:N
    cp = cos(phi(k));   sp = sin(phi(k));
    ct = cos(theta(k)); st = sin(theta(k));
    cy = cos(psi(k));   sy = sin(psi(k));
    
    R_bn = [cy*ct,  cy*st*sp - sy*cp,  cy*st*cp + sy*sp;
            sy*ct,  sy*st*sp + cy*cp,  sy*st*cp - cy*sp;
           -st,     ct*sp,              ct*cp           ];
       
    bias_body = ba(k,:)';
    bias_ned  = R_bn * bias_body;
    az_bias_true(k) = bias_ned(3); 
end

% Pack for comparison
ts_az_bias_true = timeseries(az_bias_true, t, 'Name', 'az_bias_true_mps2');