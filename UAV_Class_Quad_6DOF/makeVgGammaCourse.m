% load_quadsim.m
%
% Initializer for quadsim.mdl.
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012

function [Vg, gamma, course] = makeVgGammaCourse(vg_ned)
    Vg = sqrt(sum(vg_ned.^2));
    if Vg==0
        gamma=0;
        course=0;
    else
        gamma = -asin(vg_ned(3)/Vg);
        course = atan2(vg_ned(2),vg_ned(1));
    end
end 