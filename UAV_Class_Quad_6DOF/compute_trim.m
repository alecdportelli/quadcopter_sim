% load_quadsim.m
%
% Initializer for quadsim.mdl.
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012

function trim_throttle = compute_trim(P)
    
    % Define f_g
    f_g = P.mass * P.gravity;
    
    % Define prop term that is the same for all motors 
    PROP_TERM = P.rho * P.C_prop * P.S_prop * P.k_motor * P.k_motor;
    
    % Solve for throttle squared
    % Assumes all throttles are the same
    % Sigma forces
    throttle_squared = f_g / (4 * PROP_TERM);

    % Take square root 
    trim_throttle = sqrt(throttle_squared);
end