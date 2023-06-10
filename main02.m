% MAIN02.M
%
% Robotic systems practice: jacobian computation
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

% clean workspace
clear
close all
clc

% DH table initialization
flag_robot = 2;
% 1 - planar 2 links
% 2 - planar 3 links
% 3 - antrhopomorphic 3 links
% 4 - jaco2 7 links

switch flag_robot
    case 1
        fprintf('\n planar 2 links \n')
        % parameters
        a     = [1 .4]';
        alpha = zeros(2,1);
        d     = zeros(2,1);
        theta = [0 45/180*pi]';
    case 2
        fprintf('\n planar 3 links \n')
        % parameters
        a     = [1 .4 .2]';
        alpha = zeros(3,1);
        d     = zeros(3,1);
        theta = [45 -10 -10]'/180*pi;
    case 3
        fprintf('\n anthropomorphic 3 links \n')
        % parameters
        a     = [0 .4 .2]';
        alpha = [pi/2 0 0]';
        d     = zeros(3,1);
        theta = [0 pi/4 -pi/8]';
    case 4
        fprintf('\n jaco2 7 links \n')
        % parameters
        a     = zeros(7,1);
        alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';
        d     = [0.2755 0 -0.410 -0.0098 -0.3072 0 0.25]';
        %theta = [pi/2 0 0 pi 0 0 0]'; % vertical
        theta = [pi/4 pi/3 -pi/8 pi/3 -pi/12 pi/12 0]'; % 
end
DH = [a alpha d theta];

% Jacobian computation
J = Jacobian(DH);
