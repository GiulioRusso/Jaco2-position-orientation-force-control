% Template to visualize in V-REP the inverse kinematics algorithm developed
%          for the kinova Jaco 2 7-DOF robot
%
% Read Instructions.odt first !
%
% Do not modify any part of this file except the strings within
%    the symbols << >>
%
% G. Antonelli, Introduction to Robotics, spring 2019


function [t, q, q_act] = main_progetto_3

clear all
close all
clc

addpath('functions_coppelia/');
addpath('functions_matlab/');
% <<
%
% Put here any initialization code: DH table, gains, final position,
% cruise velocity, etc.
%
% >>

%% << Defined variables

% Port
porta = 19997;          % default V-REP port

% Time variables
Ts = 0.01;              % sampling time

time_square_side = 3;   % time to spend on the square side 
time_waiting_corner = 0.5; % time waiting on the corner
time_waiting_last_corner = 2; % time waiting on the last corner

t_side = time_square_side + time_waiting_corner; % time spent on side (3s) and waiting (0.5s)
t_side_vector = 0:Ts:t_side; % time side axis

t_final_side = time_square_side + time_waiting_last_corner; % time spent on the last side (3s) and waiting on the last corner (2s)
t_final_side_vector = 0:Ts:t_final_side; % time last side axis

tf = (time_square_side + time_waiting_corner) * 3 + time_square_side + time_waiting_last_corner; % final time = 3s + 0.5s waiting on the corner for each side (3 side) + last 3s and 2s of waiting on the last corner 
t  = 0:Ts:tf;           % time vector
N  = length(t);         % number of points of the simulation

% Joints variables
n = 7;                  % joint number
q      = zeros(n,N);    % q(:,i) collects the joint position for t(i)StartVrep
q_jaco = zeros(n,N);    % q_jaco(:,i) collects the joint position for t(i) in Kinova convention
dq     = zeros(n,N);    % q(:,i) collects the joint position for t(i)
q(:,1) = [77  -17 0  43 -94 77 71]'/180*pi; % home configuration
%q(:,1) = [2.2738 0.1837 -0.3007 1.3661 0.0583 1.1910 5.7329]'; % e.e. down configuration
q_jaco(:,1) = mask_q_DH2Jaco(q(:,1));

% DH table
DH_a = [0 0 0 0 0 0 0]';
DH_alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';
DH_d = [0.2755 0 -0.41 -0.0098 -0.3111 0 0.2638]';
DH = [DH_a DH_alpha DH_d q(:,1)];
% Initialization of DH table
DH(:,4) = q(:,1);

% Gain
kp = 10;
ko = 40;
K = diag([kp*[1 1 1] ko*[1 1 1]]); % dim. 6x6 because we control both position and orientation

% Jacobian variables
J = zeros(6,n,N);               % Jacobian
condJ = zeros(n,1);             % Condition number

% Extract the position vector of the initial configuration
T = DirectKinematics(DH);       % Direct Kinematics
start_position = T(1:3,4,n);    % Starting position vector of the robot

Ri = T(1:3,1:3,n);              % Initial e.e. rotation matrix
R = zeros(3,3,N);               % Rotation matrix

r = [0 0 1]';                   % Desired rotation around initial e.e. z axis

% Position path variables
length_square_side = 0.15;      % Length of the square side
cruise_velocity_scalar = 1.5 * (length_square_side/time_square_side);  % Cruise velocity

pd  = zeros(3,N);               % Desired position e.e.
p = zeros(3,N);                 % Position e.e. wrt base frame

dpd  = zeros(3,N);              % Desired velocity e.e. wrt base frame
dp = zeros(3,N);                % Velocity e.e. wrt base frame

% Orientation path variables
angle_rotation_side = pi/4;     % Angle of the desired rotation of the e.e.
angular_cruise_velocity = 1.5 * (angle_rotation_side/time_square_side);  % Angular cruise velocity

Rd = zeros(3,3,N);              % Desired e.e. rotation matrix during changing orientation
so  = zeros(1,N);               % Desired rotation
dso  = zeros(1,N);              % Desired angular velocity e.e. along z axis of e.e.

wd = zeros(3,N);                % Desired angular velocity e.e. wrt e.e. frame
w = zeros(3,N);                 % Angular velocity e.e. wrt e.e. frame

quat_d = zeros(4,N);            % Desired quaternion vector
quat = zeros(4,N);              % Quaternion e.e

% Error
error_pos = zeros(3,N);         % Position error
error_quat = zeros(3,N);        % Quaternion error

error = zeros(6,N);             % Total error
%% >>

clc
fprintf('----------------------');
fprintf('\n simulation started ');
fprintf('\n trying to connect...\n');
[clientID, vrep ] = StartVrep(porta, Ts);
%vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);

handle_joint = my_get_handle_Joint(vrep,clientID);      % handle to the joints
my_set_joint_target_position(vrep, clientID, handle_joint, q_jaco(:,1)); % first move to q0
%q_act(:,1) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep
% Kinova conversion -> DH
%q_act(:,1) = mask_q_DH2Jaco(q_act(:,1));

SIMULATION = true;
global joint_initialized;
joint_initialized = false;


if ~SIMULATION
    rosshutdown
    rosinit
    
    joint_pub = rospublisher('/initial_configuration','sensor_msgs/JointState');
    joint_velocity_pub = rospublisher('/desired_joint_velocity','sensor_msgs/JointState');
    action_sub = rossubscriber('/joint_init_result',@actionCB,'std_msgs/Bool','DataFormat','struct');
    q_init = mask_q_DH2Jaco(q(:,1));
    q_init = rad2deg(q_init);
    q_init_msg = rosmessage(joint_pub);
    q_init_msg.Position(1) = q_init(1);
    q_init_msg.Position(2) = q_init(2);
    q_init_msg.Position(3) = q_init(3);
    q_init_msg.Position(4) = q_init(4);
    q_init_msg.Position(5) = q_init(5);
    q_init_msg.Position(6) = q_init(6);
    q_init_msg.Position(7) = q_init(7);
    send(joint_pub,q_init_msg);
    % clear joint_pub;
    fprintf("Waiting for the robot to reach the initial joint configuration\n");
    while joint_initialized == false
        pause(0.01);
    end
    fprintf("Initial joint configuration reached\n");
end

% main simulation loop

for i=1:N
    
    t_ini_iter = clock;
      
    % <<
    %
    % Put here :
    %   computation of the desired trajectory in the operative space
    %   computation of the error
    %   computation of the desired joint velocity: dq(:,i)
    %
    % >>
    
    % next line to be commented when running his own code
    
    % dq(:,i) = ???;
    
    %% << Desired task trajectory
    
    %
    %   For each side:
    %   - move 15 cm each side -> delta = 0.15
    %   with Ts = 0.01 -> 1s = 100 steps
    %   - spent 3s = 300 steps
    %   - stop on each corner 500ms = 50 steps
    %   - trapezoidal velocity
    %   - once come back to the initial corner, stop 2s = 200 steps
    %   - rotate of 45 degrees from 1->2 around z e.e., come back to the
    %   initial orientation from 2->3, redo the same from 3->4 and 4->1
    %   
    %           2 ------ 3
    %           |        |
    %           |        |
    %           |        |
    %       z   1 ------ 4
    %       |
    %       .--y   
    %      x
    %
    
    % 1 -> 2 + waiting = +0.15 along z axis in 3s + 0.5s = 3.5s from the starting configuration    
    if i <= 350 % t = +3s
        % Starting to scroll the t_side using the j-index from 1 to 350 steps (3.5s)
        if (i == 1)
            j=1;
        end
        % Run on the side of the square in 3s in a total time of 3.5s (0.5s
        % of waiting) increasing position and cruise velocity along +z
        [pd(:,i),dpd(:,i),~] = trapezoidal(start_position, start_position + [0; 0; length_square_side], [0 0 cruise_velocity_scalar], time_square_side, t_side_vector(j));
        
        % Orientation from 0 to pi/4
        [so(i), dso(i), ~] = trapezoidal(0, angle_rotation_side, angular_cruise_velocity, time_square_side, t_side_vector(j));
        % Rotation matrix given axis rotation r, and angle so
        Rd(:,:,i) = Ri*Rot_axisangle(r,so(i));
        % Angular velocity vector wrt e.e. frame
        wd(:,i) = [0 0 dso(i)]';
        
        j=j+1; % scroll t_side with j from 1 to 350 steps (3.5s)
    
    % 2 -> 3 + waiting = +0.15 along y axis in 3s + 0.5s = 3.5s from the previous configuration
    elseif i >= 351 && i <= 700 % t = +6.5s
        % Rewind the j-index on t_side to restart from the new corner
        if (i == 351)
            j=1;
        end
        % Run on the side of the square in 3s in a total time of 3.5s (0.5s
        % of waiting) increasing position and cruise velocity along +y
        [pd(:,i),dpd(:,i),~] = trapezoidal(start_position + [0; 0; length_square_side],start_position + [0; length_square_side; length_square_side], [0 cruise_velocity_scalar 0], time_square_side, t_side_vector(j));
        
        % Orientation from pi/4 to 0
        [so(i), dso(i), ~] = trapezoidal(angle_rotation_side, 0, angular_cruise_velocity, time_square_side, t_side_vector(j));
        % Rotation matrix given axis rotation r, and angle so
        Rd(:,:,i) = Ri*Rot_axisangle(r,so(i));
        % Angular velocity vector wrt e.e. frame
        wd(:,i) = [0 0 dso(i)]';
        
        j=j+1; % scroll t_side with j from 1 to 350 steps (3.5s)

    % 3 -> 4 + waiting = +0.15 along -z axis in 3s + 0.5s = 3.5s from the previous configuration
    elseif i >= 701 && i <= 1050 % t = +10s
        % Rewind the j-index on t_side to restart from the new corner
        if (i == 701)
            j=1;
        end
        % Run on the side of the square in 3s in a total time of 3.5s (0.5s
        % of waiting) increasing position and cruise velocity along -z
        [pd(:,i),dpd(:,i),~] = trapezoidal(start_position + [0; length_square_side; length_square_side], start_position + [0; length_square_side; 0], [0 0 cruise_velocity_scalar], time_square_side, t_side_vector(j));
        
        % Orientation from 0 to pi/4
        [so(i), dso(i), ~] = trapezoidal(0, angle_rotation_side, angular_cruise_velocity, time_square_side, t_side_vector(j));
        % Rotation matrix given axis rotation r, and angle so
        Rd(:,:,i) = Ri*Rot_axisangle(r,so(i));
        % Angular velocity vector wrt e.e. frame
        wd(:,i) = [0 0 dso(i)]';
        
        j=j+1; % scroll t_side with j from 1 to 350 steps (3.5s)
    
    % 4 -> 1 + waiting = +0.15 along -y axis in 3s + 2s = 5s from the previous configuration
    elseif i >= 1051 && i <= 1551 % t = +13.5s
        % Rewind the j-index on t_side to restart from the new corner
        if (i == 1051)
            j=1;
        end
        % Run on the side of the square in 3s in a total time of 5s (2s of
        % waiting) increasing position and cruise velocity along -y
        [pd(:,i),dpd(:,i),~] = trapezoidal(start_position + [0; length_square_side; 0], start_position, [0 cruise_velocity_scalar 0], time_square_side, t_final_side_vector(j));
        
        % Orientation from pi/4 to 0
        [so(i), dso(i), ~] = trapezoidal(angle_rotation_side, 0, angular_cruise_velocity, time_square_side, t_final_side_vector(j));
        % Rotation matrix given axis rotation r, and angle so
        Rd(:,:,i) = Ri*Rot_axisangle(r,so(i));
        % Angular velocity vector wrt e.e. frame
        wd(:,i) = [0 0 dso(i)]';
        
        j=j+1; % scroll t_side with j from 1 to 350 steps (3.5s)
    end
    
    % Evaluate the current desired quaternion from the rotation matrix
    quat_d(:,i) = Rot2Quat(Rd(:,:,i));

    % Direct kinematics
    DH(:,4) = q(:,i);
    T = DirectKinematics(DH);
    % get the position vector
    p(:,i) = T(1:3,4,n);
    % get the corresponding quaternion
    R(:,:,i) = T(1:3,1:3,n); % rotation matrix
    quat(:,i) = Rot2Quat(R(:,:,i));

    % Jacobian
    J(:,:,i) = Jacobian(DH);
    % Condition number
    condJ(i) = cond(J(:,:,i));
    
    % Inverse kinematics algorithm
    % Position error
    error_pos(:,i) = pd(:,i) - p(:,i);
    % Orientation error with quaternion
    error_quat(:,i) = QuatError(quat_d(:,i), quat(:,i));
    % Error
    error(:,i) = [error_pos(:,i); error_quat(:,i)];
    
    % Joints velocities using Differential kinematics (dq = J' * (dxd + K * e) with dxd wrt base frame)
    dq(:,i) = pinv(J(:,:,i)) * ([dpd(:,i); R(:,:,i) * wd(:,i)] + K * error(:,i)); % use all the J components (Jp and Jo)
    % dpd is wrt base frame, wd is wrt e.e. frame, so it has to be
    % multiplicated by the rotation matrix that goes from e.e. to base frame
    
    % e.e. linear velocities
    dp(:,i) = J(1:3,:,i) * dq(:,i);
    % e.e. angular velocities (wrt the e.e. frame)
    w(:,i) = R(:,:,i)' * J(4:6,:,i) * dq(:,i); % rotation matrix transpose because we go from base frame to e.e. frame
    %% >>
 
    
    if VelocitySaturationJaco(dq(:,i))
        fprintf("Too high joint velocities\n");
        return
    end
    
    
    % integration
    if i<N
        q(:,i+1) = q(:,i) + Ts*dq(:,i);
    end
    % DH -> Kinova conversion
    q_jaco(:,i) = mask_q_DH2Jaco(q(:,i));
    my_set_joint_target_position(vrep, clientID, handle_joint, q_jaco(:,i));
    %q_act(:,i) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep
    % Kinova conversion -> DH
    %q_act(:,i) = mask_q_Jaco2DH(q_act(:,i));
    
    
    if ~SIMULATION
        
        dq_msg = rosmessage(joint_velocity_pub);
        dq_msg.Velocity(1) = dq(1,i);
        dq_msg.Velocity(2) = dq(2,i);
        dq_msg.Velocity(3) = dq(3,i);
        dq_msg.Velocity(4) = dq(4,i);
        dq_msg.Velocity(5) = dq(5,i);
        dq_msg.Velocity(6) = dq(6,i);
        dq_msg.Velocity(7) = dq(7,i);
        
        dq_msg.Position(1) = q_jaco(1,i);
        dq_msg.Position(2) = q_jaco(2,i);
        dq_msg.Position(3) = q_jaco(3,i);
        dq_msg.Position(4) = q_jaco(4,i);
        dq_msg.Position(5) = q_jaco(5,i);
        dq_msg.Position(6) = q_jaco(6,i);
        dq_msg.Position(7) = q_jaco(7,i);
        send(joint_velocity_pub,dq_msg);
        
    end
    
    t_fin_iter = clock;
    duration_iter = t_fin_iter(6) - t_ini_iter(6);
    pause(Ts-duration_iter)

end
    
%% << Plot
figure('Name','Robot variables');
sgtitle("$\mathbf{K_{P}}$ = " + strjoin(string(kp)) + ", $\mathbf{K_{O}}$ = " + strjoin(string(ko)), 'Interpreter','latex')

subplot(221)
plot(t,q,'LineWidth',2),grid
xticks([0 3 3.5 6.5 7 10 10.5 13.5 15.5])
ylabel('\textbf{q} [rad]', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('Joints positions')
legend('${q}_1$', '${q}_2$', '${q}_3$','${q}_4$', '${q}_5$', '${q}_6$', '${q}_7$', 'Interpreter','latex');

subplot(223)
plot(t,dq,'LineWidth',2),grid
xticks([0 3 3.5 6.5 7 10 10.5 13.5 15.5])
ylabel('\textbf{$\dot{\mathbf{q}}$ [rad/s]}', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf]), title('Joints velocities')
legend('$\dot{q}_1$', '$\dot{q}_2$', '$\dot{q}_3$','$\dot{q}_4$', '$\dot{q}_5$', '$\dot{q}_6$', '$\dot{q}_7$', 'Interpreter','latex');

subplot(2,2,[2 4])
plot(t,condJ,'LineWidth',2),grid
xticks([0 3 3.5 6.5 7 10 10.5 13.5 15.5])
ylabel('$\mathbf{\kappa(J)}$', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('Jacobian condition number')



figure('Name','Position analysis');
sgtitle("$\mathbf{K_{P}}$ = " + strjoin(string(kp)) + ", $\mathbf{K_{O}}$ = " + strjoin(string(ko)), 'Interpreter','latex')

subplot(211)
plot(t,p,'LineWidth',2),grid
xticks([0 3 3.5 6.5 7 10 10.5 13.5 15.5])
ylabel('$\mathbf{p_{e}}$ \textbf{[m]}', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('End Effector positions (wrt base frame)')
legend('$\mathbf{p_{e_{x}}}$','$\mathbf{p_{e_{y}}}$','$\mathbf{p_{e_{z}}}$', 'Interpreter','latex','Location','southeast')

subplot(212)
plot(t,pd,'LineWidth',2),grid
xticks([0 3 3.5 6.5 7 10 10.5 13.5 15.5])
ylabel('$\mathbf{p_{d}}$ \textbf{[m]}', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('End Effector desired positions (wrt base frame)')
legend('$\mathbf{p_{d_{x}}}$','$\mathbf{p_{d_{y}}}$','$\mathbf{p_{d_{z}}}$', 'Interpreter','latex','Location','southeast')



figure('Name','Velocity analysis');
sgtitle("$\mathbf{K_{P}}$ = " + strjoin(string(kp)) + ", $\mathbf{K_{O}}$ = " + strjoin(string(ko)), 'Interpreter','latex')

subplot(221)
plot(t,dp,'LineWidth',2),grid
xticks([0 3 3.5 6.5 7 10 10.5 13.5 15.5])
ylabel('$\mathbf{\dot{p_{e}}}$ \textbf{[m/s]}', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('End Effector linear velocities (wrt base frame)')
legend('$\mathbf{\dot{p_{e_{x}}}}$','$\mathbf{\dot{p_{e_{y}}}}$','$\mathbf{\dot{p_{e_{z}}}}$', 'Interpreter','latex','Location','southeast')

subplot(223)
plot(t,dpd,'LineWidth',2),grid
xticks([0 3 3.5 6.5 7 10 10.5 13.5 15.5])
ylabel('$\mathbf{\dot{p_{d}}}$ \textbf{[m/s]}', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('End Effector desired linear velocities (wrt base frame)')
legend('$\mathbf{\dot{p_{d_{x}}}}$','$\mathbf{\dot{p_{d_{y}}}}$','$\mathbf{\dot{p_{d_{z}}}}$', 'Interpreter','latex','Location','southeast')

subplot(222)
plot(t,w,'LineWidth',2),grid
xticks([0 3 3.5 6.5 7 10 10.5 13.5 15.5])
ylabel('$\mathbf{\omega_{e}}$ \textbf{[rad/s]}', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('End Effector angular velocities (wrt e.e. frame)')
legend('$\mathbf{\omega_{e_{x}}}$','$\mathbf{\omega_{e_{y}}}$','$\mathbf{\omega_{e_{z}}}$', 'Interpreter','latex','Location','southeast')

subplot(224)
plot(t,wd,'LineWidth',2),grid
xticks([0 3 3.5 6.5 7 10 10.5 13.5 15.5])
ylabel('$\mathbf{\omega_{d}}$ \textbf{[rad/s]}', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('End Effector desired angular velocities (wrt e.e. frame)')
legend('$\mathbf{\omega_{d_{x}}}$','$\mathbf{\omega_{d_{y}}}$','$\mathbf{\omega_{d_{z}}}$', 'Interpreter','latex','Location','southeast')


figure('Name','Quaternion analysis')
sgtitle("$\mathbf{K_{P}}$ = " + strjoin(string(kp)) + ", $\mathbf{K_{O}}$ = " + strjoin(string(ko)), 'Interpreter','latex')

subplot(211)
plot(t,quat,'LineWidth',2),grid
xticks([0 3 3.5 6.5 7 10 10.5 13.5 15.5])
ylabel('$\mathbf{Q}$ \textbf{[-]}', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('Quaternion')
legend('$\varepsilon_x$', '$\varepsilon_y$', '$\varepsilon_z$', '$\eta$', 'Interpreter','latex');

subplot(212)
plot(t,quat_d,'LineWidth',2),grid
xticks([0 3 3.5 6.5 7 10 10.5 13.5 15.5])
ylabel('$\mathbf{Q_{d}}$ \textbf{[-]}', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('Desired Quaternion')
legend('$\varepsilon_x$', '$\varepsilon_y$', '$\varepsilon_z$', '$\eta$', 'Interpreter','latex');


figure('Name','Error analysis');
sgtitle("$\mathbf{K_{P}}$ = " + strjoin(string(kp)) + ", $\mathbf{K_{O}}$ = " + strjoin(string(ko)), 'Interpreter','latex')

subplot(211)
plot(t,error_pos,'LineWidth',2),grid
xticks([0 3 3.5 6.5 7 10 10.5 13.5 15.5])
ylabel('$\mathbf{e_{P}}$ \textbf{[m]}', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('Cartesian position error')
legend('$\mathbf{e_{Px}}$','$\mathbf{e_{Py}}$','$\mathbf{e_{Pz}}$', 'Interpreter','latex','Location','southeast')

subplot(212)
plot(t,error_quat,'LineWidth',2),grid
xticks([0 3 3.5 6.5 7 10 10.5 13.5 15.5])
ylabel('$\mathbf{e_{O}}$ \textbf{[-]}', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('Quaternion orientation error')
legend('$\mathbf{e_{O1}}$','$\mathbf{e_{O2}}$','$\mathbf{e_{O3}}$', 'Interpreter','latex','Location','southeast')



figure('Name','Robot corner positions');

hold on
DH(:,4) = q(:,1);
subplot(151)
title('Initial position 1');
DrawRobot(DH);
view(90,0)

DH(:,4) = q(:,301);
subplot(152)
title('Position 2');
DrawRobot(DH);
view(90,0)

DH(:,4) = q(:,651);
subplot(153)
title('Position 3');
DrawRobot(DH);
view(90,0)

DH(:,4) = q(:,1001);
subplot(154)
title('Position 4');
DrawRobot(DH);
view(90,0)

DH(:,4) = q(:,N);
subplot(155)
title('Initial position 1');
DrawRobot(DH);
view(90,0)


figure('Name','Robot corner overlapped positions');

hold on
DH(:,4) = q(:,1);
DrawRobot(DH);
hold on
DH(:,4) = q(:,301);
DrawRobot(DH);
hold on
DH(:,4) = q(:,651);
DrawRobot(DH);
hold on
DH(:,4) = q(:,1001);
DrawRobot(DH);
hold on
DH(:,4) = q(:,N);
DrawRobot(DH);
%% >>

vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
DeleteVrep(clientID, vrep);

end

% constructor
function [clientID, vrep ] = StartVrep(porta, Ts)

vrep = remApi('remoteApi');   % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);        % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',porta,true,true,5000,5);% start the simulation

if (clientID>-1)
    disp('remote API server connected successfully');
else
    disp('failed connecting to remote API server');
    DeleteVrep(clientID, vrep); %call the destructor!
end
% to change the simulation step time use this command below, a custom dt in v-rep must be selected,
% and run matlab before v-rep otherwise it will not be changed
vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, Ts, vrep.simx_opmode_oneshot_wait);
vrep.simxSetBooleanParameter(clientID, vrep.sim_boolparam_realtime_simulation, true, vrep.simx_opmode_oneshot_wait);
vrep.simxSetBooleanParameter(clientID, vrep.sim_boolparam_dynamics_handling_enabled, false, vrep.simx_opmode_oneshot_wait);

vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
end

% destructor
function DeleteVrep(clientID, vrep)

vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait); % pause simulation
%vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait); % stop simulation
vrep.simxFinish(clientID);  % close the line if still open
vrep.delete();              % call the destructor!
disp('simulation ended');

end

function my_set_joint_target_position(vrep, clientID, handle_joint, q)

[m,n] = size(q);
for i=1:n
    for j=1:m
        err = vrep.simxSetJointPosition(clientID,handle_joint(j),q(j,i),vrep.simx_opmode_oneshot);
        if (err ~= vrep.simx_error_noerror)
            fprintf('failed to send joint angle q %d \n',j);
        end
    end
end

end

function handle_joint = my_get_handle_Joint(vrep,clientID)

[~,handle_joint(1)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_1',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(2)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_2',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(3)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_3',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(4)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_4',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(5)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_5',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(6)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_6',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(7)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_7',vrep.simx_opmode_oneshot_wait);

end

function my_set_joint_signal_position(vrep, clientID, q)

[~,n] = size(q);

for i=1:n
    joints_positions = vrep.simxPackFloats(q(:,i)');
    [err]=vrep.simxSetStringSignal(clientID,'jointsAngles',joints_positions,vrep.simx_opmode_oneshot_wait);
    
    if (err~=vrep.simx_return_ok)
        fprintf('failed to send the string signal of iteration %d \n',i);
    end
end
pause(8);% wait till the script receives all data, increase it if dt is too small or tf is too high

end


function angle = my_get_joint_target_position(clientID,vrep,handle_joint,n)

for j=1:n
    vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_streaming);
end

pause(0.05);

for j=1:n
    [err(j),angle(j)]=vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_buffer);
end

if (err(j)~=vrep.simx_return_ok)
    fprintf(' failed to get position of joint %d \n',j);
end

end

function [] = actionCB(handle,msg)

global joint_initialized

joint_initialized = msg.Data;

end


function saturated = VelocitySaturationJaco(dq)

joints = size(dq,1);
max_velocity = [0.6283 0.6283 0.6283 0.6283 0.8378 0.8378 0.8378]';
scale = 1;
max_velocity = max_velocity*scale;
saturated = false;

for  i=1:joints
    
    if abs(dq(i)) > max_velocity(i)
        saturated = true;
    end
end

end
