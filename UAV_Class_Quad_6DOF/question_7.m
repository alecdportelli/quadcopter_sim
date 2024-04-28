% Question 6
%
% Initializer for quadsim.mdl.
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012

% Compute trim conditions
[trim_throttle, P] = compute_trim(P);

% Linearize the system
[A, B] = linearize_quadsim(P);

% Compute the H matrix
H = ss(A, B, eye(12), zeros(12,4));
H = zpk(minreal(H));

% Extract Lat and Lon matrices
kpd =  3; ku  = 4;  kv  = 5;  kw  = 6; 
kpsi = 9; kp  = 10; kq  = 11; kr  = 12;
kde =  1; kda = 2;  kdr = 3;  kdt = 4;
kphi = 7; ktheta = 8; 


%% Transfer functions -----------------------------------------------------

s = tf('s');
% dr2yaw
dr2yaw_numerical = H(kpsi,kdr);
dr2yaw_analytical = (8 * P.delta_t0 * P.k_Tp * P.k_omega^2) / (P.Jz * s^2);

% da2phi
da2phi_numerical = H(kphi, kda);
da2phi_analytical = (8 * P.delta_t0 * P.delta_y * P.k_Pf * P.k_omega * P.k_omega) ... 
     / (P.Jx * s^2);

% de2theta
de2theta_numerical = H(ktheta,kde);
de2theta_analytical = (8 * P.delta_x * P.k_Pf * P.k_omega * P.k_omega * P.delta_t0) ... 
     / (P.Jy * s^2);

% dt2alt
dt2alt_numerical = -1*H(kpd,kdt);
dt2alt_analytical = ((8/P.mass) * P.k_Pf * P.k_omega * P.k_omega ... 
    * P.delta_t0) / (s^2);

%% ------------------------------------------------------------------------
% Step 1: Simulate the step response of the transfer function
s = tf('s');
dt2alt_analytical = ((8/P.mass) * P.k_Pf * P.k_omega * P.k_omega ... 
    * P.delta_t0) / (s^2);
t_end = max(out.time_s); % End time for the simulation
[y_tf, t_tf] = step(dt2alt_analytical, t_end);

% Step 2: Scale and shift the transfer function response
scaling_factor = max(abs(out.alt_rate_mps)) / max(abs(y_tf)); % Scaling factor
shift_factor = 0; % Shift factor (adjust as needed)
y_tf_scaled = scaling_factor * y_tf + shift_factor; % Scaled and shifted transfer function response

dt2alt_numerical = -1*H(kpd,kdt);

% Step 3: Plot the comparison
figure;
plot(out.time_s, out.alt_rate_mps, 'b', 'LineWidth', 2); % Plot 6DOF output
hold on;
plot(t_tf, y_tf_scaled, 'r', 'LineWidth', 2); % Plot scaled and shifted transfer function response
hold on;
plot(t_tf, dt2alt_numerical, 'g--', 'LineWidth', 2); % Plot scaled and shifted transfer function response
xlabel('Time (s)');
ylabel('Altitude Rate (m/s)');
title('Comparison of Transfer Function with 6DOF Output');
legend('6DOF Output', 'Analytical' , 'Numerical');
grid on;
