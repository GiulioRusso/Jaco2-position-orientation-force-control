% MAIN09.M
%
% Robotic systems practice: differential kinematic inversion
% on a "rich" trajectory
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

% clean workspace
clear
close all
clc

% DH table initialization
fprintf('\n jaco2 7 links \n')
% parameters
a     = zeros(7,1);
alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';
d     = [0.2755 0 -0.410 -0.0098 -0.3072 0 0.25]';
theta = [77  -17 0  43 -94 77 71]'/180*pi; % approximated home configuration
%theta = [pi/2 0 0 pi 0 0 0]'; % vertical
theta = [45 -50 -15 90 150 25 -90]'/180*pi;
DH = [a alpha d theta];

% IK gain
K = diag([20*[1 1 1], 10*[1 1 1]]);

% inizialite simulation variables
tf = 15;
Ts = 1e-2;
n = 7;
t = 0:Ts:tf;
N = length(t);
xd  = zeros(3,N);
x   = zeros(3,N);
q   = zeros(n,N);
q(:,1) = theta;

dq  = zeros(n,N);
quat = zeros(4,N);
quat_d = zeros(4,N);
error_pos = zeros(3,N);
error_quat = zeros(3,N);
error = zeros(6,N);
condJ = zeros(1,N);

% desired trajectory
R = eye(3);             % orientamento circonferenza rispetto alla terna base
rho = 0.1;              % raggio della circonferenza
T = DirectKinematics(DH);
c = T(1:3,4,n) - rho*[1 0 0]'; % centro circonferenza = posizione - raggio
Ri = T(1:3,1:3,n);      % Rotazione e.e. iniziale
r = [0 0 1]';           % Rotazione desiderata attorno all'asse z
thetaf = -360/180*pi;   % Rotazione orientamento finale desiderato

for i=1:N
    %% desired task trajectory
    % Ascissa curvilinea
    % slide 48 robotics05.pdf
    %
    % Posizione:
    % Il profilo che voglio assegnare parte da 0 e deve compiere un giro di
    % circonferenza, pertanto la posizione finale sarà 2*pi*raggio. Assegno
    % una velocità di crociera e un tempo finale, e passo l'istante
    % corrente
    [sp, ~, ~] = trapezoidal(0,2*pi*rho,.1,8,t(i));
    
    %xd(:,i) = c + R*rho*[cos(sp/rho); sin(sp/rho); 0]; % sul piano xy
    xd(:,i) = c + R*rho*[cos(sp/rho); 0; sin(sp/rho)]; % sul piano xz
    %xd(:,i) = c + R*rho*[0; cos(sp/rho); sin(sp/rho)]; % sul piano yz
    %
    % Orientamento:
    % Assegno un profilo trapezoidale anche all'orientamento con
    % orientamento finale desiderato thetaf
    [so, dso, ~] = trapezoidal(0,thetaf,1,8,t(i));
    
    % slide 50 robotics05.pdf
    Rd = Ri*Rot_axisangle(r,so);
    %quat_d(:,i) = [0 0 0 1]';
    quat_d(:,i) = Rot2Quat(Rd);
    %% direct kinematics
    DH(:,4) = q(:,i);
    T = DirectKinematics(DH);
    % Posizione e.e.
    x(:,i) = T(1:3,4,n);
    % Quaternione dalla matrice di rotazione
    quat(:,i) = Rot2Quat(T(1:3,1:3,n));
    % Jacobian
    J = Jacobian(DH);
    condJ(i) = cond(J);
    %% Inverse kinematics algorithm
    error_pos(:,i) = xd(:,i) - x(:,i);
    error_quat(:,i) = QuatError(quat_d(:,i),quat(:,i));
    error(:,i) = [error_pos(:,i);error_quat(:,i)];
        
    dq(:,i) = pinv_damped(J,eye(n))*K*error(:,i);
    if i<N
        q(:,i+1) = q(:,i) + Ts*dq(:,i);
    end
end

figure
plot3(xd(1,:),xd(2,:),xd(3,:))
grid, axis equal

figure
subplot(231)
plot(t,q),grid
ylabel('q [rad]')
subplot(234)
plot(t,dq),grid
ylabel('dq [rad/s]')
subplot(232)
plot(t,error(1:2,:)),grid
ylabel('p err [m]')
subplot(235)
plot(t,error(3:5,:)),grid
ylabel('o err [-]')
xlabel('time [s]')
subplot(233)
plot(t,condJ),grid
ylabel('cond J')
xlabel('time [s]')

figure
hold on
DH(:,4) = q(:,1);
DrawRobot(DH);
DH(:,4) = q(:,N);
DrawRobot(DH);

