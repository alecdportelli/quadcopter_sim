function [A, B] = linearize_quadsim(P)
    % Create a linearized state space model of the uavsim aircraft about the
    % nominal conditions in the input structure P.

    % We'll linearize about a nominal state vector (P.x0) and control 
    % deflections (P.delta0). It is presumed that P.x0 is the trimmed state 
    % vector, and P.delta0 is the trimmed control deflections.
    x0 =  [ ...
            P.pn0;    ...
            P.pe0;    ...
            P.pd0;    ...
            P.u0;     ...
            P.v0;     ...
            P.w0;     ...
            P.phi0;   ...
            P.theta0; ...
            P.psi0;   ...
            P.p0;     ...
            P.q0;     ...
            P.r0;     ...
        ];
    u0 = [P.delta_e0; P.delta_a0; P.delta_r0; P.delta_t0];

    % Evaluate state derivatives at nominal condition.
    %   xdot0 = f(x0,u0)
    xdot0 = eval_forces_moments_kin_dyn(x0,u0,P);

    % Construct linearized A matrix one column at a time
    A=zeros(length(x0));
    eps_perturb=1e-8; % Amount to perturb each state
    for i=1:length(x0)
        % Compute the ith column of A
        %   - Perturb the ith state (and only the ith state) by adding eps_perturb
        x_perturbed = x0;
        x_perturbed(i) = x_perturbed(i) + eps_perturb;
        %   - Eval the perturbed xdot: xdot_perturbed = f(x_perturbed,u0) 
        xdot_perturbed = eval_forces_moments_kin_dyn(x_perturbed, u0, P);
        %   - A(:,i) = ( f(x_perturbed,u0) - f(x0,u0) ) / eps_perturb
        A(:,i) = (xdot_perturbed - xdot0) / eps_perturb;
    end

    % Construct linearized B matrix a column at a time
    B=zeros(length(x0), length(u0));
    eps_perturb=1e-8; % Amount to perturb each deflection
    for i=1:length(u0)
        % Compute the ith column of B
        %   - Perturb the ith control deflection (and only the ith deflection) by adding eps_perturb
        u_perturbed = u0;
        u_perturbed(i) = u_perturbed(i) + eps_perturb;
        %   - Eval the perturbed xdot: xdot_perturbed = f(x0,u_perturbed) 
        xdot_perturbed = eval_forces_moments_kin_dyn(x0, u_perturbed, P);
        %   - B(:,i) = ( f(x0,u_perturbed) - f(x0,u0) ) / eps_perturb
        B(:,i) = (xdot_perturbed - xdot0) / eps_perturb;
    end

    % Linearization is a function of wind. Notify user if non-zero wind
    % condition.
    if any([P.wind_n; P.wind_e; P.wind_d]~=0)
        disp('NOTE: Linearization performed about non-zero wind condition')
    end
    
end

function xdot = eval_forces_moments_kin_dyn(x,deltas,P)
    % Other inputs needed
    wind_ned = [P.wind_n; P.wind_e; P.wind_d];
    time=0;

    % f_and_m = uav_forces_moments(uu, P)
    %   INPUT: uu = [wind_ned(1:3); deltas(1:4); x(1:12); time(1)];
    %   OUTPUT: out = [Forces; Torques]; % Length 3+3=6
    uu = [wind_ned(1:3); deltas(1:4); x(1:12); time];
    f_and_m = quadsim_forces_moments(uu, P);

    % xdot = uav_kin_dyn(uu,P)
    %   INPUT: uu = [x(1:12); f_and_m(1:6); time(1)];
    %   OUTPUT: xdot = [Pdot_ned; vgdot_b; euler_rates; wdot_b];
    uu = [x(1:12); f_and_m(1:6); time];
    xdot = quadsim_kin_dyn(uu, P);
end
