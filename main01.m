% MAIN01.M
%
% Robotic systems practice: direct kinematics computation
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

% clear the workspace
clear
close all
clc

% DH table initialization
flag_robot = 3;
% 1 - planar 2 links
% 2 - planar 3 links
% 3 - antrhopomorphic 3 links
% 4 - jaco2 7 links

% inizializzo la tabella DH in base al robot che scelgo
switch flag_robot
    case 1
        fprintf('\n planar 2 links \n')
        % parameters
        a     = [1 .4]';
        alpha = zeros(2,1);
        d     = zeros(2,1);
        theta = [-10 45]'/180*pi;
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
        a     = [.1 .4 .2]';
        alpha = [pi/2 0 0]';
        d     = zeros(3,1);
        theta = [0 pi/4 -pi/8]';
    case 4
        fprintf('\n jaco2 7 links \n')
        % parameters
        a     = zeros(7,1);
        alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';
        d     = [0.2755 0 -0.410 -0.0098 -0.3072 0 0.25]';
        theta = [pi/4 pi/3 -pi/8 pi/3 -pi/12 pi/12 0]';
end
DH = [a alpha d theta];

% direct kinematics computation
T0 = DirectKinematics(DH)

% draw robot
DrawRobot(DH)

%{

 anthropomorphic 3 links 

 Matrice di trasformazione omogenea dalla terna 0 alla terna 1
 T0(:,:,1) =

    1.0000         0         0    0.1000
         0    0.0000   -1.0000         0
         0    1.0000    0.0000         0
         0         0         0    1.0000
 

 
 Matrice di rotazione dalla terna 0 alla terna 1
   0 ---------- 0
sistema      sistema
 x/y/z       x'/y'/z'

 R = 

     1.0000         0         0     =   proiezioni di x' su x/y/z
          0    0.0000   -1.0000     =   proiezioni di y' su x/y/z
          0    1.0000    0.0000     =   proiezioni di z' su x/y/z

 r(i,j) = 1 il versore i si sovrappone in maniera concorde al versore j
 r(i,j) = -1 il versore i si sovrappone in maniera discorde al versore j
 r(i,j) = 0 il versore i NON si sovrappone al versore j



 Coordinate cartesiane dell'origine del sistema di riferimento x'/y'/z'
 p = 

     0.1000
          0
     0.0000 



 riga costante = 

    0         0         0    1.0000



 E così via ...

 Matrice di trasformazione omogenea dalla terna 0 alla terna 2
 T0(:,:,2) = 
    
    0.7071   -0.7071         0    0.3828
    0.0000    0.0000   -1.0000    0.0000
    0.7071    0.7071    0.0000    0.2828
         0         0         0    1.0000



 Matrice di rotazione dalla terna 0 alla terna 1
    0 ---------- 0 ------------ 0
 sistema                     sistema
  x/y/z                      x'/y'/z'
 
 R = 

    0.7071   -0.7071         0
    0.0000    0.0000   -1.0000
    0.7071    0.7071    0.0000



 Coordinate cartesiane dell'origine del sistema di riferimento x'/y'/z'
 p = 

    0.3828
    0.0000
    0.2828



 riga costante = 

    0         0         0    1.0000

...
%}

