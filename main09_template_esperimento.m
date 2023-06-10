% Template to visualize in V-REP the inverse kinematics algorithm developed
%          for the kinova Jaco 2 7-DOF robot
%
% Read Instructions.odt first !
%
% Do not modify any part of this file except the strings within
%    the symbols << >>
%
% G. Antonelli, Introduction to Robotics, spring 2019


function [t, q, q_act] = main_template

close all
clc

addpath('functions_coppelia/');
addpath('functions_matlab/');
porta = 19997;          % default V-REP port
tf = 12;                % final time
Ts = 0.01;              % sampling time
t  = 0:Ts:tf;           % time vector
N  = length(t);         % number of points of the simulation
n = 7;                  % joint number
q      = zeros(n,N);    % q(:,i) collects the joint position for t(i)StartVrep
q_jaco = zeros(n,N);    % q_jaco(:,i) collects the joint position for t(i) in Kinova convention
dq     = zeros(n,N);    % q(:,i) collects the joint position for t(i)
q(:,1) = [2.2738 0.1837 -0.3007 1.3661 0.0583 1.1910 5.7329]';   % initial configuration

q_jaco(:,1) = mask_q_DH2Jaco(q(:,1));

% <<
%
% Put here any initialization code: DH table, gains, final position,
% cruise velocity, etc.
%
% >>

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

DH_a = [0 0 0 0 0 0 0]';
DH_alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';
DH_d = [0.2755 0 -0.41 -0.0098 -0.3111 0 0.2638]';
DH = [DH_a DH_alpha DH_d q(:,1)];

DrawRobot(DH)
pause;

K = diag([15*[1 1 1], 25*[1 1 1]]);


SIMULATION = true;
% false: solo a Coppelia
% true: anche tramite ROS
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

DH(:,4) = q(:,1);

%% << Variabili aggiunte
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
%% >> Fine Variabili aggiunte

for i=1:N
    
    t_ini_iter = clock;

    
    %% << Calcolo dq
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
    %xd(:,i) = c + R*rho*[cos(sp/rho); 0; sin(sp/rho)]; % sul piano xz
    xd(:,i) = c + R*rho*[0; cos(sp/rho); sin(sp/rho)]; % sul piano yz
    %
    % Orientamento:
    % Assegno un profilo trapezoidale anche all'orientamento con
    % orientamento finale desiderato thetaf
    [so, ~, ~] = trapezoidal(0,thetaf,1,8,t(i));
    
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
    %% >> Calcolo dq
    
    %{
    if VelocitySaturationJaco(dq(:,i))
        fprintf("Too high joint velocities\n");
        return
    end
    %}
    
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
