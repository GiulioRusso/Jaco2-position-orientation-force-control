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

K = diag([15*[1 1 1], 25*[1 1 1]]);


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

DH(:,4) = q(:,1);



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
    
    
    dq(:,i) = ???;
 
    
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
