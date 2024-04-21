% load_quadsim.m
%
% Initializer for quadsim.mdl.
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012

function [phi_rad theta_rad psi_rad] = rotationMatrixToEuler(R)
    if abs(R(1,3))==1
        phi_rad = 0;
        theta_rad = -asin(R(1,3));
        psi_rad = -atan2(R(2,1),R(2,2));
    else
        phi_rad = atan2(R(2,3),R(3,3));
        theta_rad = -asin(R(1,3));
        psi_rad = atan2(R(1,2),R(1,1));
    end
    % NOTE: Returns in radians
end