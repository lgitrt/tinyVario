%% trajectory_paraglider.m  (ENHANCED — v2.2 Kinematically Fixed)
%
% Generates realistic paraglider flight trajectories as MATLAB timeseries.
% Completely base-MATLAB compatible (No Toolbox Dependencies).
%
% ── COORDINATE FRAME ────────────────────────────────────────────────────
%   World : NED  (X=North, Y=East, Z=Down)
%   Body  : X=forward, Y=right, Z=down
%   Euler : ZYX convention  →  psi(yaw), theta(pitch), phi(roll)  [rad]
%
% Run param_init.m first to load 'param'.
% ────────────────────────────────────────────────────────────────────────
if ~exist('param','var')
    param_init;
end
rng(42);   % reproducible noise
%% =========================================================
%  CONSTANTS & TIME VECTOR
% ==========================================================
dt   = param.dt_imu;
t    = (0:dt:1180)';
N    = length(t);
g    = param.simul.g;
%% =========================================================
%  AIRSPEED MODEL
% ==========================================================
v_trim   = 10.0;   % [m/s]
v_stall0 =  7.0;   % [m/s]
v_bar    = 13.0;   % [m/s]
v_brakes =  8.5;   % [m/s]
% Per-phase target airspeed (Matched exactly to 19 phases)
phase_vs = {'trim','brake','speedbar','trim','brake','speedbar',...
            'speedbar','brake','speedbar','brake','speedbar',...
            'brake','speedbar','brake','speedbar',...
            'trim','brake','trim','brake'};
speed_map = struct('trim',v_trim,'brake',v_brakes,'speedbar',v_bar);
%% =========================================================
%  FLIGHT PHASE DEFINITIONS
% ==========================================================
phases = [
%  t_start  t_end   vz_mean  bank_deg  hdg_rate_dps
    0,       15,     3.5,     0,        0;       % 1  launch
   15,      120,     2.8,    18,        5;       % 2  thermal1
  120,      180,    -0.8,     5,       -2;       % 3  glide1
  180,      240,    -0.5,   -22,       -6;       % 4  turn1
  240,      360,     3.2,    20,        5;       % 5  thermal2
  360,      420,    -1.0,     0,        0;       % 6  glide2
  420,      450,    -0.5,     0,        0;       % 7  inter-glide
  450,      570,     4.0,    45,        9.8;     % 8  tight_thermal1
  570,      600,    -0.8,     5,        1;       % 9  exit glide 1
  600,      720,     4.8,   -50,      -11.7;     % 10 tight_thermal2
  720,      750,    -0.8,    -5,       -1;       % 11 exit glide 2
  750,      870,     5.0,    55,       14.0;     % 12 tight_thermal3
  870,      900,    -1.5,     0,        0;       % 13 exit glide 3
  900,      920,    -0.2,   -10,       -2;       % 14 weak thermal entry
  920,     1040,     0.5,   -22,       -4.0;     % 15 weak_thermal
 1040,     1060,    -0.5,    -5,       -1;       % 16 weak exit
 1060,     1100,    -0.8,   -15,       -4;       % 17 approach turn
 1100,     1150,    -1.5,     5,        1;       % 18 approach
 1150,     1180,    -2.0,     0,        0;       % 19 landing flare
];
n_phases = size(phases,1);
%% =========================================================
%  WIND FIELD & GUSTS
% ==========================================================
v_ref  = [3.0, 1.5];   % Reference wind [N, E] at h_ref
h_ref  = 500;
alpha  = 0.14;
gust_N = zeros(N,1);  gust_E = zeros(N,1);
sigma_gust = 0.8;   tau_gust = 8.0;
for k = 2:N
    gust_N(k) = gust_N(k-1)*(1 - dt/tau_gust) + sigma_gust*sqrt(2*dt/tau_gust)*randn();
    gust_E(k) = gust_E(k-1)*(1 - dt/tau_gust) + sigma_gust*sqrt(2*dt/tau_gust)*randn();
end
%% =========================================================
%  THERMAL SPATIAL STRUCTURE
% ==========================================================
thermal_phases = [8, 10, 12, 15];
thermal_r_offset = [15, 12, 10, 25];
thermal_r_core   = [30, 25, 20, 50];
thermal_dVz_edge = [1.2, 1.5, 1.8, 0.3];
%% =========================================================
%  COLLAPSE EVENT MODEL (Knuth Poisson Implementation)
% ==========================================================
collapse_events = [];
for ph = [8, 10, 12]
    t0_ph = phases(ph,1);  t1_ph = phases(ph,2);
    ph_len = t1_ph - t0_ph;
    
    % Poisson random variable generation via Knuth's algorithm
    L_poiss = exp(-1.2); k_poiss = 0; p_poiss = 1;
    while p_poiss > L_poiss
        k_poiss = k_poiss + 1;
        p_poiss = p_poiss * rand();
    end
    n_collapses = k_poiss - 1;
    
    for c = 1:n_collapses
        tc = t0_ph + 5 + rand()*(ph_len - 10);
        dur = 3 + rand()*4;
        spike = (15 + rand()*25) * sign(phases(ph,4));
        vz_drop = -(1.5 + rand()*2.5);
        collapse_events(end+1,:) = [tc, dur, spike, vz_drop]; %#ok<SAGROW>
    end
end
%% =========================================================
%  INITIALISE STATE VECTORS
% ==========================================================
vz_world = zeros(N,1);
alt      = zeros(N,1);   alt(1) = 800;
phi      = zeros(N,1);   phi(1) = 0;
theta    = zeros(N,1);   theta(1) = 2*pi/180;
psi      = zeros(N,1);   psi(1) = 0;
v_air_ts = zeros(N,1);   v_air_ts(1) = v_trim;
vN       = zeros(N,1);
vE       = zeros(N,1);
phi_correction = zeros(N,1);
tau_phi_corr   = 4.0;   sigma_phi_corr = 6*pi/180;
% Turbulence and Wind Tracking States (Smooth Inertial Response)
vz_turb       = 0;     tau_turb = 3.0; sigma_turb = 0.4;
wind_N_glider = 0;     wind_E_glider = 0; tau_wind_lag = 1.5;
%% =========================================================
%  MAIN INTEGRATION LOOP
% ==========================================================
tau_vz_default  = 5.0;
tau_phi_default = 2.0;
tau_phi_tight   = 1.2;
tau_v_air       = 4.0;
theta_0         = 2*pi/180;
k_v_pitch       = -0.008;
flare_h         = 5.0;
for k = 2:N
    tk = t(k);
    % 1. Identify Phase
    ph_idx = find(phases(:,1) <= tk & phases(:,2) > tk, 1, 'last');
    if isempty(ph_idx), ph_idx = n_phases; end
    vz_target    = phases(ph_idx, 3);
    bank_target  = phases(ph_idx, 4) * pi/180;
    hdg_rate     = phases(ph_idx, 5) * pi/180;
    % 2. Collapse Processing
    col_vz_mod  = 0;
    col_phi_mod = 0;
    if ~isempty(collapse_events)
        for c = 1:size(collapse_events,1)
            tc  = collapse_events(c,1);
            dur = collapse_events(c,2);
            if tk >= tc && tk < tc + dur
                frac = (tk - tc)/dur;
                spike_shape = 4*frac*(1-frac);
                col_phi_mod = col_phi_mod + collapse_events(c,3)*pi/180 * spike_shape;
                col_vz_mod  = col_vz_mod  + collapse_events(c,4) * (1 - frac);
            end
        end
    end
    % 3. Thermal Spatial Structure
    th_mask = find(thermal_phases == ph_idx, 1);
    vz_spatial = 0;
    if ~isempty(th_mask)
        gauss_w = exp(-0.5*(thermal_r_offset(th_mask)/thermal_r_core(th_mask))^2);
        vz_spatial = thermal_dVz_edge(th_mask) * gauss_w * sin(psi(k-1));
    end
    % 4. Smooth Vertical Speed Integration
    vz_turb = vz_turb * (1 - dt/tau_turb) + sigma_turb * sqrt(2*dt/tau_turb) * randn();
    tau_vz  = tau_vz_default;
    vz_world(k) = vz_world(k-1) + dt/tau_vz * ...
                  (vz_target + vz_spatial + col_vz_mod + vz_turb - vz_world(k-1));
    % 5. Airspeed Handling
    vs_key = phase_vs{ph_idx};
    v_target_raw = speed_map.(vs_key);
    phi_now = phi(k-1);
    v_stall_banked = v_stall0 / sqrt(max(cos(phi_now), 0.1));
    v_target = max(v_target_raw, 1.1 * v_stall_banked);
    v_target = max(v_target - 0.3 * max(vz_world(k), 0), v_stall_banked * 1.05);
    v_air_ts(k) = v_air_ts(k-1) + dt/tau_v_air * (v_target - v_air_ts(k-1));
    v_air_ts(k) = max(v_air_ts(k), v_stall_banked * 1.02);
    % 6. Bank Angle (Collapse offsets injected directly into pilot control loop)
    tau_phi = tau_phi_default; if abs(phases(ph_idx,4)) >= 45, tau_phi = tau_phi_tight; end
    
    phi_correction(k) = phi_correction(k-1)*(1 - dt/tau_phi_corr) + ...
                        sigma_phi_corr * sqrt(2*dt/tau_phi_corr) * randn();
    is_thermalling = ismember(ph_idx, [2,5,8,10,12,15]);
    corr_scale = 0.4 * is_thermalling + 0.1 * (~is_thermalling);
    phi(k) = phi(k-1) + dt/tau_phi * ((bank_target + col_phi_mod) - phi(k-1)) + ...
             corr_scale * phi_correction(k) * dt;
    % 7. Heading
    if abs(bank_target) > 0.05
        hdg_rate_actual = g * tan(phi(k)) / v_air_ts(k);
    else
        hdg_rate_actual = hdg_rate;
    end
    psi(k) = psi(k-1) + dt * hdg_rate_actual;
    % 8. Coupled Altitude & Landing Flare
    alt(k) = alt(k-1) + dt * vz_world(k);
    if alt(k) <= flare_h && alt(k) > 0
        vz_world(k) = vz_world(k) * (alt(k) / flare_h)^2;
        alt(k) = max(alt(k-1) + dt * vz_world(k), 0);
    else
        alt(k) = max(alt(k), 0);
    end
    % 9. Pitch Coupling
    theta_vz   = asin(max(-0.3, min(0.3, vz_world(k) / v_air_ts(k))));
    theta_turn = abs(phi(k)) * 0.15;
    theta_spd  = k_v_pitch * (v_air_ts(k) - v_trim);
    theta_cmd  = theta_0 + theta_spd + theta_vz + theta_turn;
    theta(k)   = 0.7*theta(k-1) + 0.3*theta_cmd;
    % 10. Wind Lag Integration (Smooth Aerodynamic Glide Profile)
    h_now  = max(alt(k), 1);
    shear  = (h_now / h_ref)^alpha;
    wind_N_target = v_ref(1) * shear + gust_N(k);
    wind_E_target = v_ref(2) * shear + gust_E(k);
    wind_N_glider = wind_N_glider + dt/tau_wind_lag * (wind_N_target - wind_N_glider);
    wind_E_glider = wind_E_glider + dt/tau_wind_lag * (wind_E_target - wind_E_glider);
    vN(k) = v_air_ts(k) * cos(psi(k)) * cos(theta(k)) + wind_N_glider;
    vE(k) = v_air_ts(k) * sin(psi(k)) * cos(theta(k)) + wind_E_glider;
end
%% =========================================================
%  ANGULAR RATES (Kinematic Axis Matrix Restored to ZYX)
% ==========================================================
dphi   = [0; diff(phi)]   / dt;
dtheta = [0; diff(theta)] / dt;
dpsi   = [0; diff(psi)]   / dt;
p   = zeros(N,1);
q_r = zeros(N,1);
r   = zeros(N,1);
for k = 1:N
    % RECONSTRUCTED MATRIX: Converts Euler derivatives directly to Body Rates [p; q; r]
    T = [ 1,         0,       -sin(theta(k));
          0,  cos(phi(k)),  sin(phi(k))*cos(theta(k));
          0, -sin(phi(k)),  cos(phi(k))*cos(theta(k)) ];
    omega = T * [dphi(k); dtheta(k); dpsi(k)];
    p(k)   = omega(1);
    q_r(k) = omega(2);
    r(k)   = omega(3);
end
%% =========================================================
%  SPECIFIC FORCE & GROUND TRUTH GENERATION
% ==========================================================
ax_body = zeros(N,1); ay_body = zeros(N,1); az_body = zeros(N,1);
dvN = [0; diff(vN)] / dt; dvE = [0; diff(vE)] / dt; dvz = [0; diff(vz_world)] / dt;
a_ned = [dvN, dvE, -dvz]; g_ned = [0, 0, g];
az_world_true = zeros(N,1);
for k = 1:N
    cp = cos(phi(k));   sp = sin(phi(k));
    ct = cos(theta(k)); st = sin(theta(k));
    cy = cos(psi(k));   sy = sin(psi(k));
    R_bn = [cy*ct,   cy*st*sp - sy*cp,   cy*st*cp + sy*sp;
            sy*ct,   sy*st*sp + cy*cp,   sy*st*cp - cy*sp;
           -st,      ct*sp,              ct*cp           ];
    f_ned      = a_ned(k,:) - g_ned;
    f_body_vec = R_bn' * f_ned';
    ax_body(k) = f_body_vec(1);
    ay_body(k) = f_body_vec(2);
    az_body(k) = f_body_vec(3);
    az_world_true(k) = f_ned(3);
end
% Cross-check validation
az_world_true_B = -sin(theta).*ax_body + cos(theta).*sin(phi).*ay_body + cos(theta).*cos(phi).*az_body;
fprintf('az_world ground truth validation error = %.2e m/s²\n', max(abs(az_world_true - az_world_true_B)));
%% =========================================================
%  IMU SENSOR NOISE APPLICATION
% ==========================================================
f_vib = 15; vib_amp = 0.2;
vibration = vib_amp * sin(2*pi*f_vib*t) .* randn(N,1);
sigma_turb2 = 0.15; tau_turb2 = 2.0; turbulence = zeros(N,1);
for k = 2:N
    turbulence(k) = turbulence(k-1)*(1 - dt/tau_turb2) + sigma_turb2 * sqrt(2*dt/tau_turb2) * randn();
end
T0 = param.baro.T0; L = param.baro.L; p0 = param.baro.p0;
T_air = max(T0 - L*alt, 200);
ba_temp_z = -0.002 * (T_air - T0);
sa = param.imu.sigma_a; sg = param.imu.sigma_g;
ba = cumsum(param.imu.bias_instability_a * sqrt(dt) * randn(N,3), 1);
bg = cumsum(param.imu.bias_instability_g * sqrt(dt) * randn(N,3), 1);
ba(:,3) = ba(:,3) + ba_temp_z;
ax_meas = max(-16*g, min(16*g, ax_body + ba(:,1) + sa*randn(N,1) + vibration + turbulence));
ay_meas = max(-16*g, min(16*g, ay_body + ba(:,2) + sa*randn(N,1) + vibration + turbulence));
az_meas = max(-16*g, min(16*g, az_body + ba(:,3) + sa*randn(N,1) + vibration + turbulence));
gx_meas = p   + bg(:,1) + sg*randn(N,1);
gy_meas = q_r + bg(:,2) + sg*randn(N,1);
gz_meas = r   + bg(:,3) + sg*randn(N,1);
%% =========================================================
%  BAROMETER INVERSION
% ==========================================================
t_baro = (0:param.dt_baro:t(end))';
N_baro = length(t_baro);
exponent = g / (287.058 * L);
alt_baro_true = interp1(t, alt, t_baro, 'linear', 'extrap');
p_baro_true   = p0 .* (1 - L * alt_baro_true / T0).^exponent;
p_baro_meas   = p_baro_true + param.baro.sigma_p * randn(N_baro,1);
alt_baro_meas = (T0 / L) .* (1 - (p_baro_meas / p0).^(1/exponent));
alt_baro_imu = interp1(t_baro, alt_baro_meas, t, 'previous', 'extrap');
baro_new = zeros(N,1); [~, baro_idx] = unique(round(t / param.dt_baro)); baro_new(baro_idx) = 1;
%% =========================================================
%  PACK INTO TIMESERIES
% ==========================================================
psi_wrapped   = mod(psi   + pi, 2*pi) - pi;
phi_wrapped   = mod(phi   + pi, 2*pi) - pi;
theta_wrapped = mod(theta + pi, 2*pi) - pi;
posN = cumsum(vN) * dt; posE = cumsum(vE) * dt;
ts_alt      = timeseries(alt,            t, 'Name','alt_true_m');
ts_vz       = timeseries(vz_world,       t, 'Name','vz_true_mps');
ts_v_air    = timeseries(v_air_ts,       t, 'Name','v_air_mps');
ts_phi_wr   = timeseries(phi_wrapped,    t, 'Name','phi_roll_rad_wr');
ts_theta_wr = timeseries(theta_wrapped,  t, 'Name','theta_pitch_rad_wr');
ts_psi_wr   = timeseries(psi_wrapped,    t, 'Name','psi_yaw_rad_wr');
ts_p        = timeseries(p,              t, 'Name','p_rollrate_radps');
ts_q        = timeseries(q_r,            t, 'Name','q_pitchrate_radps');
ts_r        = timeseries(r,              t, 'Name','r_yawrate_radps');
ts_px       = timeseries(posN,           t, 'Name','px_body_meas_m');
ts_py       = timeseries(posE,           t, 'Name','py_body_meas_m');
ts_pz       = timeseries(alt,            t, 'Name','pz_body_meas_m');
ts_ax       = timeseries(ax_meas,        t, 'Name','ax_body_meas_mps2');
ts_ay       = timeseries(ay_meas,        t, 'Name','ay_body_meas_mps2');
ts_az       = timeseries(az_meas,        t, 'Name','az_body_meas_mps2');
ts_gx       = timeseries(gx_meas,        t, 'Name','gx_body_meas_radps');
ts_gy       = timeseries(gy_meas,        t, 'Name','gy_body_meas_radps');
ts_gz       = timeseries(gz_meas,        t, 'Name','gz_body_meas_radps');
ts_alt_baro = timeseries(alt_baro_imu,   t, 'Name','alt_baro_meas_m');
ts_baro_new = timeseries(baro_new,       t, 'Name','baro_new_flag');
ts_az_world_true = timeseries(az_world_true, t, 'Name','az_world_true_mps2'); 
%% =========================================================
%  DIAGNOSTIC VISUALIZATION
% ==========================================================
figure('Name','Paraglider Bug-Fixed Trajectory (v2.2)','NumberTitle','off','Position',[50 50 1500 850]);
subplot(2,2,1); plot(t, alt, 'k', t, alt_baro_imu, 'r--'); grid on; title('Altitude Data Tracking'); legend('True','Baro');
subplot(2,2,2); plot(t, phi_wrapped*180/pi, 'b', t, theta_wrapped*180/pi, 'g'); grid on; title('Attitude Dynamics');
subplot(2,2,3); plot(t, ax_meas, 'b', t, az_meas, 'k'); grid on; title('Pristine IMU Acceleration Fields');
subplot(2,2,4); plot(posE, posN, 'm'); axis equal; grid on; title('Ground Track Navigation Blueprint');
sgtitle('Paraglider Simulation Environment v2.2 (Restored ZYX Framework)');