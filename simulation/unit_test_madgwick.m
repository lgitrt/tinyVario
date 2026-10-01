%% unit_test_madgwick.m
% Traces the Madgwick objective function with known exact values.
% Fixed physical conventions for NED (Right roll = negative ay reaction).
clear;
clc;
fprintf('=== Madgwick unit test ===\n\n');
g = 9.80665;

%% =========================================================
%  TEST 1 — Static check at identity quaternion
%  At rest, level, body Z down:
%    accel reads [0, 0, -g]  (reaction to gravity)
% ==========================================================
fprintf('--- TEST 1: identity quaternion, sensor at rest ---\n');
q = [1 0 0 0];   % identity: body aligned with NED
ax = 0; ay = 0; az = -g;   % body Z down, sensor reads -g

% Normalise
an = sqrt(ax^2+ay^2+az^2);
axn=ax/an; ayn=ay/an; azn=az/an;
fprintf('  axn=%.4f  ayn=%.4f  azn=%.4f  (expect 0, 0, -1)\n', axn,ayn,azn);

w=q(1); x=q(2); y=q(3); z=q(4);

% Objective function (F = R_eb * g_ned - a_body)
F1 =  2*(w*y - x*z) - axn;
F2 = -2*(w*x + y*z) - ayn;
F3 = -(w^2 - x^2 - y^2 + z^2) - azn;

fprintf('  F1=%.4f  F2=%.4f  F3=%.4f  (all must be 0)\n', F1,F2,F3);
if abs(F1)<1e-9 && abs(F2)<1e-9 && abs(F3)<1e-9
    fprintf('  TEST 1: PASS\n\n');
else
    fprintf('  TEST 1: FAIL — objective is nonzero at rest!\n\n');
end

% Gradient at rest
gw =  2*y*F1 - 2*x*F2 - 2*w*F3;
gx_= -2*z*F1 - 2*w*F2 + 2*x*F3;
gy_=  2*w*F1 - 2*z*F2 + 2*y*F3;
gz_= -2*x*F1 - 2*y*F2 - 2*z*F3;
fprintf('  Gradient at rest: [%.6f %.6f %.6f %.6f] (all must be 0)\n\n',gw,gx_,gy_,gz_);

% az_world at rest (Body-to-Earth Row 3)
R31 = 2*(x*z - w*y); 
R32 = 2*(y*z + w*x); 
R33 = w^2 - x^2 - y^2 + z^2;
az_world = R31*ax + R32*ay + R33*az;

fprintf('  az_world=%.4f  (expect %.4f = -g)\n', az_world, -g);
fprintf('  a_up = -(az_world+g) = %.4f  (expect 0)\n\n', -(az_world+g));

%% =========================================================
%  TEST 2 — 10 deg nose-up pitch, sensor at rest
% ==========================================================
fprintf('--- TEST 2: 10 deg pitch-up, at rest ---\n');
theta_deg = 10;
theta_rad = theta_deg * pi/180;
q_pitch = [cos(theta_rad/2), 0, sin(theta_rad/2), 0];

ax_expect =  sin(theta_rad) * g;
ay_expect =  0;
az_expect = -cos(theta_rad) * g;

ax=ax_expect; ay=ay_expect; az=az_expect;
an=sqrt(ax^2+ay^2+az^2);
axn=ax/an; ayn=ay/an; azn=az/an;

w=q_pitch(1); x=q_pitch(2); y=q_pitch(3); z=q_pitch(4);
F1 =  2*(w*y - x*z) - axn; 
F2 = -2*(w*x + y*z) - ayn; 
F3 = -(w^2 - x^2 - y^2 + z^2) - azn;

fprintf('  F = [%.6f, %.6f, %.6f]  (all must be 0)\n', F1,F2,F3);
if abs(F1)<1e-6 && abs(F2)<1e-6 && abs(F3)<1e-6
    fprintf('  TEST 2: PASS\n\n');
else
    fprintf('  TEST 2: FAIL\n\n');
end

%% =========================================================
%  TEST 3 — Gradient direction test (5-deg roll-right)
%  PHYSICS FIX: Right roll moves Body-Y toward ground. 
%  Upward reaction force projects as NEGATIVE Body-Y.
% ==========================================================
fprintf('--- TEST 3: gradient direction for 5-deg roll ---\n');
roll_deg = 5;
roll_rad = roll_deg * pi/180;

ax_tilt =  0;
ay_tilt = -sin(roll_rad) * g; % Reaction is UP, Sensor tilted RIGHT => ay is negative
az_tilt = -cos(roll_rad) * g;

an = sqrt(ax_tilt^2+ay_tilt^2+az_tilt^2);
axn=ax_tilt/an; ayn=ay_tilt/an; azn=az_tilt/an;
fprintf('  Accel for 5-deg right roll (normalised): [%.4f, %.4f, %.4f]\n',axn,ayn,azn);

q = [1 0 0 0];
w=q(1); x=q(2); y=q(3); z=q(4);
F1 =  2*(w*y - x*z) - axn; 
F2 = -2*(w*x + y*z) - ayn; 
F3 = -(w^2 - x^2 - y^2 + z^2) - azn;

gw =  2*y*F1 - 2*x*F2 - 2*w*F3;
gx_= -2*z*F1 - 2*w*F2 + 2*x*F3;
gy_=  2*w*F1 - 2*z*F2 + 2*y*F3;
gz_= -2*x*F1 - 2*y*F2 - 2*z*F3;

gn=sqrt(gw^2+gx_^2+gy_^2+gz_^2); if gn<1e-10, gn=1; end
gx_n = gx_/gn;

fprintf('  Normalised grad_x = %.4f  (Must be NEGATIVE for Right-Roll convergence)\n', gx_n);
if gx_n < 0
    fprintf('  TEST 3: PASS — Gradient direction is correct\n\n');
else
    fprintf('  TEST 3: FAIL — Sign error in gradient\n\n');
end

%% =========================================================
%  TEST 4 — 50-step convergence (Roll)
% ==========================================================
fprintf('--- TEST 4: 50-step convergence from identity, 5-deg roll ---\n');
q   = [1 0 0 0];
beta_test = 0.1;
dt_test   = 1/52;
ax=ax_tilt; ay=ay_tilt; az=az_tilt;
an=sqrt(ax^2+ay^2+az^2);
axn=ax/an; ayn=ay/an; azn=az/an;

for step = 1:50
    w=q(1); x=q(2); y=q(3); z=q(4);
    F1 =  2*(w*y - x*z) - axn; 
    F2 = -2*(w*x + y*z) - ayn; 
    F3 = -(w^2 - x^2 - y^2 + z^2) - azn;
    gw =  2*y*F1 - 2*x*F2 - 2*w*F3;
    gx_= -2*z*F1 - 2*w*F2 + 2*x*F3;
    gy_=  2*w*F1 - 2*z*F2 + 2*y*F3;
    gz_= -2*x*F1 - 2*y*F2 - 2*z*F3;
    gn=sqrt(gw^2+gx_^2+gy_^2+gz_^2); if gn<1e-10,gn=1; end
    qdw=-beta_test*(gw/gn); qdx=-beta_test*(gx_/gn); 
    qdy=-beta_test*(gy_/gn); qdz=-beta_test*(gz_/gn);
    w=w+qdw*dt_test; x=x+qdx*dt_test; y=y+qdy*dt_test; z=z+qdz*dt_test;
    qn=sqrt(w^2+x^2+y^2+z^2); q=[w x y z]/qn;
end

roll_converged = atan2(2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2)) * 180/pi;
fprintf('  After 50 steps, roll = %.2f deg (expect +5.0)\n', roll_converged);
if abs(roll_converged - 5.0) < 1.0, fprintf('  TEST 4: PASS\n\n'); else, fprintf('  TEST 4: FAIL\n\n'); end

%% =========================================================
%  TEST 5 — 50-step convergence (Pitch)
% ==========================================================
fprintf('--- TEST 5: 50-step convergence from identity, 5-deg pitch-up ---\n');
pitch_deg = 5;
pitch_rad = pitch_deg * pi/180;
ax_p = sin(pitch_rad)*g; ay_p = 0; az_p = -cos(pitch_rad)*g;
an=sqrt(ax_p^2+ay_p^2+az_p^2); axn=ax_p/an; ayn=ay_p/an; azn=az_p/an;

q=[1 0 0 0];
for step=1:50
    w=q(1); x=q(2); y=q(3); z=q(4);
    F1 =  2*(w*y - x*z) - axn; 
    F2 = -2*(w*x + y*z) - ayn; 
    F3 = -(w^2 - x^2 - y^2 + z^2) - azn;
    gw =  2*y*F1 - 2*x*F2 - 2*w*F3;
    gx_= -2*z*F1 - 2*w*F2 + 2*x*F3;
    gy_=  2*w*F1 - 2*z*F2 + 2*y*F3;
    gz_= -2*x*F1 - 2*y*F2 - 2*z*F3;
    gn=sqrt(gw^2+gx_^2+gy_^2+gz_^2); if gn<1e-10,gn=1; end
    q = q - beta_test * (dt_test * [gw, gx_, gy_, gz_]/gn);
    q = q / norm(q);
end

pitch_converged = asin(max(-1,min(1,2*(q(1)*q(3)-q(4)*q(2))))) * 180/pi;
fprintf('  After 50 steps, pitch = %.2f deg (expect +5.0)\n', pitch_converged);
if abs(pitch_converged - 5.0) < 1.0, fprintf('  TEST 5: PASS\n\n'); else, fprintf('  TEST 5: FAIL\n\n'); end

%% =========================================================
%  TEST 6 — az_world during a known tilt
% ==========================================================
fprintf('--- TEST 6: az_world at 10-deg pitch, at rest ---\n');
q = [cos(theta_rad/2), 0, sin(theta_rad/2), 0]; 
ax = sin(theta_rad)*g; ay = 0; az = -cos(theta_rad)*g;
w=q(1); x=q(2); y=q(3); z=q(4);

R31 = 2*(x*z - w*y); 
R32 = 2*(y*z + w*x); 
R33 = w^2 - x^2 - y^2 + z^2;
az_world = R31*ax + R32*ay + R33*az;

fprintf('  az_world = %.6f (expect %.6f)\n', az_world, -g);
if abs(az_world - (-g)) < 1e-6, fprintf('  TEST 6: PASS\n\n'); else, fprintf('  TEST 6: FAIL\n\n'); end

fprintf('=== All tests completed ===\n');