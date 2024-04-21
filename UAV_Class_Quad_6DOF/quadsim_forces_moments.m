% quadsim_forces_moments.m
%
% Generation of forces and moments acting on vehicle for quadsim
%
% Inputs:
%   Wind in NED frame
%   Control surfaces
%   UAV States
%   Time
%
% Outputs:
%   Forces in body frame
%   Moments in body frame
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   
function out = quadsim_forces_moments(uu, P)

    % Extract variables from input vector uu
    %   uu = [wind_ned(1:3); deltas(1:4); x(1:12); time(1)];
    k=(1:3);          wind_ned=uu(k);   % Total wind vector, ned, m/s
    k=k(end)+(1:4);   deltas=uu(k);     % Control surface commands: [delta_e delta_a delta_r delta_t]
    k=k(end)+(1:12);  x=uu(k);          % states
    k=k(end)+(1);     time=uu(k);       % Simulation time, s

    % Extract state variables from x
    pn    = x(1);   % North position, m
    pe    = x(2);   % East position, m
    pd    = x(3);   % Down position, m
    u     = x(4);   % body-x groundspeed component, m/s
    v     = x(5);   % body-y groundspeed component, m/s
    w     = x(6);   % body-z groundspeed component, m/s
    phi   = x(7);   % EulerAngle: roll, rad
    theta = x(8);   % EulerAngle: pitch, rad
    psi   = x(9);   % EulerAngle: yaw, rad
    p     = x(10);  % body rate about x, rad/s
    q     = x(11);  % body rate about y, rad/s
    r     = x(12);  % body rate about z, rad/s

    % Combine states to vector form for convenience
    P_ned = [pn; pe; pd];   % NED position, m
    vg_b  = [u; v; w];      % Groundspeed vector, body frame, m/s
    w_b   = [p; q; r];      % body rates about x,y,z, rad/s

    % Extract control commands from deltas
    delta_e = deltas(1); % Elevator, +/-
    delta_a = deltas(2); % Aileron, +/-
    delta_r = deltas(3); % Rudder, +/-
    delta_t = deltas(4); % Throttle, 0 - 1
    [delta_1, delta_2, delta_3, delta_4] = mapChannelsToMotors(delta_e,delta_a,delta_r,delta_t);

    %% ADDED CODE:
    % Add the local frame
    R_ned2b = eulerToRotationMatrix(phi,theta,psi);

    % Prop rotation rates
    omega_1 = (P.k_omega*delta_1) + P.prop_1_omega_bias;  
    omega_2 = (P.k_omega*delta_2) + P.prop_2_omega_bias;
    omega_3 = (P.k_omega*delta_3) + P.prop_3_omega_bias;
    omega_4 = (P.k_omega*delta_4) + P.prop_4_omega_bias;

    % Define k hat
    i_hat = R_ned2b(:, 1);
    j_hat = R_ned2b(:, 2);
    k_hat = R_ned2b(:, 3);

    % Define V_air_in 
    % Compute the wind velocity vector in the body frame
    wind_b = R_ned2b * wind_ned;

    % compute airspeed Va, angle-of-attack alpha, side-slip beta
    [Va, alpha, beta] = makeVaAlphaBeta(vg_b - wind_b);

    % Compute the air coming in
    V_air_in = dot((vg_b - wind_b), (-k_hat));

    % Define prop term 
    PROP_TERM = P.rho * P.C_prop * P.S_prop;

    % Motor terms 
    MOTOR_TERM_1 = ((omega_1 / P.k_omega) * (P.k_motor - V_air_in));
    MOTOR_TERM_2 = ((omega_2 / P.k_omega) * (P.k_motor - V_air_in));
    MOTOR_TERM_3 = ((omega_3 / P.k_omega) * (P.k_motor - V_air_in));
    MOTOR_TERM_4 = ((omega_4 / P.k_omega) * (P.k_motor - V_air_in));

    % Compute forces 
    F1 = PROP_TERM * (V_air_in + MOTOR_TERM_1) * (MOTOR_TERM_1);
    F2 = PROP_TERM * (V_air_in + MOTOR_TERM_2) * (MOTOR_TERM_2);
    F3 = PROP_TERM * (V_air_in + MOTOR_TERM_3) * (MOTOR_TERM_3);
    F4 = PROP_TERM * (V_air_in + MOTOR_TERM_4) * (MOTOR_TERM_4);


    % Compute torques  
    T1 = P.k_Tp * (omega_1*omega_1); 
    T2 = P.k_Tp * (omega_1*omega_1); 
    T3 = P.k_Tp * (omega_1*omega_1); 
    T4 = P.k_Tp * (omega_1*omega_1); 

    % Compute moments
    M_x = ((P.delta_y*(F3+F2)) - (P.delta_y*(F1+F4))) * i_hat;
    M_y = ((P.delta_x*(F1+F3)) - (P.delta_x*(F2+F4))) * j_hat;
    M_z = ((T1 + T2) - (T3 + T4)) * k_hat;

    % Assemble F prop
    f_prop_b = -1 * (F1 + F2 + F3 + F4) * k_hat;

    % Define force from gravity on the body
    f_grav_b = P.mass * R_ned2b * [0; 0; P.gravity];

    % Define forces from the motor drap
    % NOTE: wind_b = R_ned2b * wind_ned;
    f_drag_b = -P.mu_rotorDrag * ...
        [vg_b(1) - wind_b(1); vg_b(2) - wind_b(2); 0];
    
    % Assemble 
    f_b = f_prop_b + f_grav_b + f_drag_b;
    m_b = M_x + M_y + M_z;
    
    % Compile function output
    out = [f_b; m_b]; % Length 3+3=6
    
end
