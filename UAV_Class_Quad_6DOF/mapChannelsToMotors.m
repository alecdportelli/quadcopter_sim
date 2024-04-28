% load_quadsim.m
%
% Initializer for quadsim.mdl.
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012

function [delta_1, delta_2, delta_3, delta_4] = mapChannelsToMotors(delta_e,delta_a,delta_r,delta_t)
% Map quadcopter channels to motors
%
% Inputs:
%    delta_e: Elevator
%    delta_a: Aileron
%    delta_r: Rudder
%    delta_t: Throttle
%
% Outputs:
%    delta_1: front right motor
%    delta_2: back left motor
%    delta_3: front left motor
%    delta_4: back right motor

    % Map channels to motors
    %     3   1    
    %       X
    %     2   4

    M_channel_motors = [1/4, -1/4, 1/4, -1/4;
                        -1/4, 1/4, 1/4, -1/4;
                        1/4,  1/4, -1/4, -1/4;
                        1/4,  1/4, 1/4,  1/4;];

    motor_singal_gain = inv(M_channel_motors) ... 
        * [delta_e; delta_a; delta_r; delta_t;];
    
    delta_1 = motor_singal_gain(1); % front right
    delta_2 = motor_singal_gain(2); % back left
    delta_3 = motor_singal_gain(3); % front left 
    delta_4 = motor_singal_gain(4); % back right
end