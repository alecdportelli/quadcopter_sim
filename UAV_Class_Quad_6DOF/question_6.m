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

disp("-----------------------------------------------")
A_lon = A([ku kw kq ktheta kpd], [ku kw kq ktheta kpd])
B_lon = B([ku kw kq ktheta kpd], [kde kdt])
A_lat = A([kv kp kr kphi kpsi], [kv kp kr kphi kpsi])
B_lat = B([kv kp kr kphi kpsi], [kda kdr])
disp("-----------------------------------------------")


%% dr2yaw -----------------------------------------------------------------

% Declare analytical TF
s = tf('s');
dr2yaw_analytical = (8 * P.delta_t0 * P.k_Tp * P.k_omega^2) / (P.Jz * s^2);
 
% Declare numerical TF
dr2yaw_numerical = H(kpsi,kdr);
 
% Plot
figure
plot(out.time_s, step(0.001*dr2yaw_analytical, out.time_s), 'b', ...
      out.time_s, step(0.001*dr2yaw_numerical, out.time_s), 'r--', 'LineWidth', 3);
legend('Analytical', 'Numerical');
xlabel('Time (s)'); % Label for the x-axis
ylabel('Response (rad/s^2)'); % Label for the y-axis
title('dr2yaw Step Response Comparison'); % Title for the plot

% -------------------------------------------------------------------------


%% da2phi -----------------------------------------------------------------

% Declare analytical TF
s = tf('s');
da2phi_analytical = (8 * P.delta_t0 * P.delta_y * P.k_Pf * P.k_omega * P.k_omega) ... 
     / (P.Jx * s^2);
 
% Declare numerical TF
da2phi_numerical = H(kphi, kda);
 
% Plot
figure
plot(out.time_s, step(0.001*da2phi_analytical, out.time_s), 'b', ...
      out.time_s, step(0.001*da2phi_numerical, out.time_s), 'r--', 'LineWidth', 3);
legend('Analytical', 'Numerical');
xlabel('Time (s)'); % Label for the x-axis
ylabel('Response (rad/s^2)'); % Label for the y-axis
title('da2phi Step Response Comparison'); % Title for the plot
%--------------------------------------------------------------------------


%% de2theta----------------------------------------------------------------
 
% Declare analytical TF
s = tf('s');
de2theta_analytical = (8 * P.delta_x * P.k_Pf * P.k_omega * P.k_omega * P.delta_t0) ... 
     / (P.Jy * s^2);
 
% Declare numerical TF
de2theta_numerical = H(ktheta,kde);
 
% Plot
figure
plot(out.time_s, step(0.001*de2theta_analytical, out.time_s), 'b', ...
      out.time_s, step(0.001*de2theta_numerical, out.time_s), 'r--', 'LineWidth', 3);
legend('Analytical', 'Numerical');
xlabel('Time (s)'); % Label for the x-axis
ylabel('Response (rad/s^2)'); % Label for the y-axis
title('de2theta Step Response Comparison'); % Title for the plot
% -------------------------------------------------------------------------
 

%% dt2alt -----------------------------------------------------------------
 
% Declare analytical TF
s = tf('s');
dt2alt_analytical = ((8/P.mass) * P.k_Pf * P.k_omega * P.k_omega ... 
    * P.delta_t0) / (s^2);
 
% Declare numerical TF
dt2alt_numerical = -1*H(kpd,kdt);
 
% Plot
figure
plot(out.time_s, step(0.001*dt2alt_analytical, out.time_s), 'b', ...
     out.time_s, step(0.001*dt2alt_numerical, out.time_s), 'r--', 'LineWidth', 3);
legend('Analytical', 'Numerical');
xlabel('Time (s)'); % Label for the x-axis
ylabel('Response (rad/s^2)'); % Label for the y-axis
title('dt2alt Step Response Comparison'); % Title for the plot