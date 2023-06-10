clear all
close all
clc

%% << Defined variables

% Time variables
Ts = 0.001;     % sampling time
tf = 30;        % final time
t = 0:Ts:tf;    % time vector
N = length(t);  % simulation points

% Joints variables
n = 7;    % joint number

q = zeros(n,N);         % Joints positions
dq = zeros(n,N);        % Joints velocities
ddq = zeros(n,N);       % Joints accelerations

q(:,1) = [77  -17 0  43 -94 77 71]'/180*pi; % home configuration
%q(:,1) = [2.2738 0.1837 -0.3007 1.3661 0.0583 1.1910 5.7329]'; % e.e. down configuration

% DH table
DH_a = [0 0 0 0 0 0 0]';
DH_alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';
DH_d = [0.2755 0 -0.41 -0.0098 -0.3111 0 0.2638]';
DH = [DH_a DH_alpha DH_d q(:,1)]; % Initialization of DH table

% Jacobian variable
J = zeros(6,n,N);       % Jacobian

JAd = zeros(6,n,N);     % Analytical Jacobian wrt desired
dJAd = zeros(6,n,N);    % Derived Analytical Jacobian wrt desired

condJ = zeros(n,1);     % Condition number

% Extract the initial position and orientation of the e.e.
T0 = DirectKinematics(DH);

% Rest e.e. position, rotation matrix and orientation
o_r = T0(1:3,4,n);      % Rest position e.e. wrt base frame
R_r = T0(1:3,1:3,n);    % Rest orientation e.e. wrt base frame
phi_r = Rot2rpy(R_r);   % Rest orientation e.e. wrt base frame

% e.e. position, rotation matrix and orientation for each point of the
% simulation
o_e   = zeros(3,N);     % Position e.e wrt base frame
R_e = zeros(3,3,N);     % Rotation matrix e.e.
phi_e = zeros(3,N);     % Orientation e.e. wrt base frame

o_d_e = zeros(3,N);     % Position e.e wrt desired frame

% e.e. position, rotation matrix and orientation for each point of the
% simulation wrt the desired frame
R_d = R_r;              % Desired rotation matrix e.e. same as the initial one wrt base frame

o_d_de = zeros(3,N);    % Position error e.e. - desired wrt desired frame
phi_de = zeros(3,N);    % Orientation error e.e. - desired wrt desired frame
R_d_e = zeros(3,3,N);   % Rotation matrix to go from the desired frame to the e.e. frame

% Desired trajectory
pd = zeros(3,N);        % Desired position wrt base frame
dpd = zeros(3,N);       % Desired velocity wrt base frame
ddpd = zeros(3,N);      % Desired acceleration wrt base frame

pd_d = zeros(3,N);        % Desired position wrt desired frame
dpd_d = zeros(3,N);       % Desired velocity wrt desired frame
ddpd_d = zeros(3,N);      % Desired acceleration wrt desired frame

% Error
x_tilde  = zeros(6,N);  % Error (position and orientation) in operational space wrt desired
dx_tilde  = zeros(6,N); % Error dynamic in operational space wrt desired

% Control vairables
xe = zeros(6,N);        % Position and Orientation e.e. wrt base frame
xr = zeros(6,N);        % Position and Orientation rest wrt base frame

h_e = zeros(6,N);       % Interaction e.e. -> env.
h_d_e = zeros(6,N);     % Interaction e.e. -> env. wrt desired frame

u = zeros(n,N);         % Joints cuples
y = zeros(n,N);         % Control Law

t_push = 10;             % Time of the pushing movement of the e.e.
z_push = 0.2;           % Distance of the movement of the e.e. along z axis

% Impedance control validation

% Desired position to reach with the push wrt base frame
% - o_r wrt base frame
% - push vector [0 0 z_push]' wrt e.e. frame
% since we want all expressed in base frame, this vector has to be
% multiplicated by the rotation matrix
o_d = o_r + R_r * [0 0 z_push]';
phi_d = zeros(3,N);     % Desired orientation wrt base frame

% Cruise velocity
% - also the cruise velocity have to be mapped into the base frame
cruise_velocity_vector = R_r * [0 0 1.5 * z_push/t_push]';

%
% e.e. push along it's z axis  
%                        _            _           _
%                      -|_          -|_         -|_
%                    o_r,phi_r   o_e,phi_e   o_d,phi_d   
%
%
%
%      .
%  base frame
%

% Gains
md = 100;
Md = md * diag([1 1 1 1 1 1]);      % Virtual mass

kd = 100;
Kd = kd * diag([1 1 1 1 1 1]);      % Damping gain (have to dump along all the directions) 

kp = 1;
Kp = kp * diag([1 1 1 1 1 1]);      % Proportional gain

kz = 100;
K = diag([0 0 kz 0 0 0]);           % Environment resistence (only along z e.e.)

kv = 1;
K_v = kv * diag([1 1 1 1 1 1 1]);   % Damping joints velocities
%% >>

for i=1:N
    
    % Desired trajectory profile (all wrt base frame)
    [pd(:,i), dpd(:,i), ddpd(:,i)] = trapezoidal(o_r, o_d, cruise_velocity_vector, t_push, t(i));
    % Desired e.e. orientation (wrt base frame)
    phi_d(:,i) = Rot2rpy(R_d); % have to remain constant
    
    % Mapping desired trajectory profile in desired frame (just for plot)
    pd_d(:,i) = R_d' * pd(:,i);
    dpd_d(:,i) = R_d' * dpd(:,i);
    ddpd_d(:,i) = R_d' * ddpd(:,i);
    
    % Direct kinematics
    DH(:,4) = q(:,i);
    T = DirectKinematics(DH);
    
    % Jacobian
    J(:,:,i) = Jacobian(DH);
    % Condition number
    condJ(i) = cond(J(:,:,i));
    
    % get the current position (wrt base frame)
    o_e(:,i) = T(1:3,4,n);
    % get the current orientation (e.e. -> base frame)
    R_e(:,:,i) = T(1:3,1:3,n);
    % e.e. current orientation (wrt base frame)
    phi_e(:,i) = Rot2rpy(R_e(:,:,i));
    
    % e.e. current position wrt desired frame (just for plot)
    o_d_e(:,i) = R_d' * o_e(:,i);
    
    % Position vector between desired and e.e. wrt desired frame
    % pag. 377 (9.8)
    o_d_de(:,i) = R_d' * (o_e(:,i) - pd(:,i));
    
    % Rotation matrix between desired and e.e. wrt desired frame
    % pag. 377 (9.8)
    R_d_e(:,:,i) = R_d' * R_e(:,:,i);
    
    % Orientation e.e. wrt desired frame
    % pag. 377 (9.8)
    phi_de(:,i) = Rot2rpy(R_d_e(:,:,i));
    
    % Evaluate error wrt desired frame
    % pag. 377 (9.9)
    x_tilde(:,i) = - [o_d_de(:,i); phi_de(:,i)];
    
    % Evaluate the Analytical Jacobain wrt desired
    TA = ComputeTA(phi_de(:,i),"rpy");
    
    % Analytical Jacobian wrt desired and it's derivative
    % JAd pag. 378 (9.14)
    %                    Rd'| 0      Jp
    % JAd =  (TA)^-1  *  -------  *  --
    %                     0 | Rd'    Jo
    JAd(:,:,i) = inv(TA) * blkdiag(R_d', R_d') * J(:,:,i);
    if i > 1
        dJAd(:,:,i) = (JAd(:,:,i) - JAd(:,:,i-1))/Ts;
    end
    
    % Error dynamic
    % pag. 378 (9.13)
    dx_tilde(:,i) = - JAd(:,:,i) * dq(:,i);
    
    % e.e. position and orientation
    xe(:,i) = [o_e(:,i); phi_e(:,i)];
    % rest position and orientation
    xr(:,i) = [o_r; phi_r];

    % e.e. forces wrt base frame
    h_e(1:3,i) = R_r * K(1:3,1:3) * (o_r - o_e(:,i));
    % (o_r - o_e(:,i)) is already in base frame
    % R_r * K(1:3,1:3) = K gain remapped in base frame
    % ! with (o_e(:,i) - o_r) the e.e. go back and the h_e is negative
    
    % e.e. forces wrt desired frame
    h_d_e(1:3,i) = R_d' * h_e(1:3,i);
    % R_d' map from the base frame to the desired frame
    
    % y signal pag 384 (9.35) (omitted: - Md * dJAd(:,:,i) * dq(:,i) + Md * db(:,i))
    y(:,i)= pinv(JAd(1:3,:,i)) * inv(Md(1:3,1:3)) * (Kd(1:3,1:3) * dx_tilde(1:3,i) + Kp(1:3,1:3) * x_tilde(1:3,i) - h_d_e(1:3,i));
    % take only the position componens of JAd, Md, Kd, dx_tilde, Kp, x_tilde, h_d_e
    
    % u signal pag. 383 (9.31)
    u(:,i) = jaco2_B(q(:,i)) * y(:,i) + jaco2_n(q(:,i),dq(:,i)) + J(1:3,:,i)' * h_e(1:3,i) - K_v * dq(:,i);
    % take only the position componens of J, h_e
    % adding damping on joints velocities due to the operational space control with a redundancy robot
    
    % Joints acceleration
    ddq(:,i) = jaco2_ddq(u(:,i), q(:,i), dq(:,i));
    
    % Integration
    if i<N
        dq(:,i+1) = dq(:,i) + Ts*ddq(:,i);
        q(:,i+1)  = q(:,i) + Ts*dq(:,i);
    end
    
    clc
    fprintf('Percentuale di completamento %.2f \n',(i/N)*100)
    
end
%% << Plot
figure('Name','Robot variables');
sgtitle("$\mathbf{M_{d}}$ = " + strjoin(string(md)) + ", $\mathbf{K_{P}}$ = " + strjoin(string(kp)) + ", $\mathbf{K_{D}}$ = " + strjoin(string(kd)) + ", $\mathbf{k_{z}}$ = " + strjoin(string(kz)) + ", $\mathbf{K_{V}}$ = " + strjoin(string(kv)) , 'Interpreter','latex')

subplot(321)
plot(t,q,'LineWidth',2),grid
xticks([0 t_push tf])
ylabel('\textbf{q [rad]}', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('Joints positions')
legend('${q}_1$', '${q}_2$', '${q}_3$','${q}_4$', '${q}_5$', '${q}_6$', '${q}_7$', 'Interpreter','latex');

subplot(323)
plot(t,dq,'LineWidth',2),grid
xticks([0 t_push tf])
ylabel('\textbf{$\dot{\mathbf{q}}$ [rad/s]}', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('Joints velocities')
legend('$\dot{q}_1$', '$\dot{q}_2$', '$\dot{q}_3$','$\dot{q}_4$', '$\dot{q}_5$', '$\dot{q}_6$', '$\dot{q}_7$', 'Interpreter','latex');

subplot(325)
plot(t,ddq,'LineWidth',2),grid
xticks([0 t_push tf])
ylabel('\textbf{$\ddot{\mathbf{q}}$ [rad/s$^2$]}', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('Joints accelerations')
legend('$\ddot{q}_1$', '$\ddot{q}_2$', '$\ddot{q}_3$','$\ddot{q}_4$', '$\ddot{q}_5$', '$\ddot{q}_6$', '$\ddot{q}_7$', 'Interpreter','latex');

subplot(3,2,[2 4 6])
plot(t,condJ,'LineWidth',2),grid
xticks([0 t_push tf])
ylabel('$\mathbf{\kappa(J)}$', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('Jacobian condition number')


figure('Name','Control variables');
sgtitle("$\mathbf{M_{d}}$ = " + strjoin(string(md)) + ", $\mathbf{K_{P}}$ = " + strjoin(string(kp)) + ", $\mathbf{K_{D}}$ = " + strjoin(string(kd)) + ", $\mathbf{k_{z}}$ = " + strjoin(string(kz)) + ", $\mathbf{K_{V}}$ = " + strjoin(string(kv)) , 'Interpreter','latex')

subplot(221)
plot(t,u,'LineWidth',2),grid
xticks([0 t_push tf])
ylabel('$\mathbf{\tau [N/m]}$', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('Joints couples')
legend({'$\mathbf{\tau_{1}}$', '$\mathbf{\tau_{2}}$', '$\mathbf{\tau_{3}}$', '$\mathbf{\tau_{4}}$', '$\mathbf{\tau_{5}}$', '$\mathbf{\tau_{6}}$', '$\mathbf{\tau_{7}}$'}, 'Interpreter', 'latex');

subplot(223)
plot(t,y,'LineWidth',2),grid
xticks([0 t_push tf])
ylabel('$\mathbf{y [m/s^{2}]}$', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('Auxiliary signal')
legend({'$\mathbf{y_{1}}$', '$\mathbf{y_{2}}$', '$\mathbf{y_{3}}$', '$\mathbf{y_{4}}$', '$\mathbf{y_{5}}$', '$\mathbf{y_{6}}$', '$\mathbf{y_{7}}$'}, 'Interpreter', 'latex');

subplot(222)
plot(t,h_d_e(1:3,:),'LineWidth',2),grid
xticks([0 t_push tf])
ylabel('$\mathbf{h_{e}^{d} [N/m]}$', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('End Effector forces (wrt desired frame)')
legend({'$\mathbf{f_{x}}$', '$\mathbf{f_{y}}$', '$\mathbf{f_{z}}$'}, 'Interpreter', 'latex');

subplot(224)
plot(t,h_e(1:3,:),'LineWidth',2),grid
xticks([0 t_push tf])
ylabel('$\mathbf{h_{e} [N/m]}$', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('End Effector forces (wrt base frame)')
legend({'$\mathbf{f_{x}}$', '$\mathbf{f_{y}}$', '$\mathbf{f_{z}}$'}, 'Interpreter', 'latex');


figure('Name','End Effector analysis');
sgtitle("$\mathbf{M_{d}}$ = " + strjoin(string(md)) + ", $\mathbf{K_{P}}$ = " + strjoin(string(kp)) + ", $\mathbf{K_{D}}$ = " + strjoin(string(kd)) + ", $\mathbf{k_{z}}$ = " + strjoin(string(kz)) + ", $\mathbf{K_{V}}$ = " + strjoin(string(kv)) , 'Interpreter','latex')

subplot(221)
plot(t,o_d_e,'LineWidth',2),grid on
xticks([0 t_push tf])
ylabel('$\mathbf{p_{e}^{d} [m]}$', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('End Effector position (wrt desired frame)')
legend('x','y','z', 'Interpreter','latex','Location','southeast')

subplot(223)
plot(t,pd_d,'LineWidth',2),grid on
xticks([0 t_push tf])
ylabel('$\mathbf{p_{d}^{d} [m]}$', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('End Effector desired position (wrt desired frame)')
legend('x','y','z', 'Interpreter','latex','Location','southeast')

subplot(222)
plot(t,x_tilde(1:3,:),'LineWidth',2),grid
xticks([0 t_push tf])
ylabel('$\mathbf{\tilde{x} [m]}$', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('Error position (wrt desired frame)')
legend('x','y','z', 'Interpreter','latex','Location','southeast')

subplot(224)
plot(t,dx_tilde(1:3,:),'LineWidth',2),grid
xticks([0 t_push tf])
ylabel('$\mathbf{\dot{\tilde{x}} [m/s]}$', 'Interpreter','latex')
xlabel('\textbf{time [s]}', 'Interpreter','latex')
xlim([0 tf])
title('Error dynamic (wrt desired frame)')
legend('x','y','z', 'Interpreter','latex','Location','southeast')



figure('Name','Robot configuration');
sgtitle("$\mathbf{M_{d}}$ = " + strjoin(string(md)) + ", $\mathbf{K_{P}}$ = " + strjoin(string(kp)) + ", $\mathbf{K_{D}}$ = " + strjoin(string(kd)) + ", $\mathbf{k_{z}}$ = " + strjoin(string(kz)) + ", $\mathbf{K_{V}}$ = " + strjoin(string(kv)) , 'Interpreter','latex')

subplot(121)
DrawRobotImp([DH_a DH_alpha DH_d q(:,1)], 1, o_r, 1, o_d, 1, o_e(:,1), 0);
view([-30 35]);
title('Rest position');
hold on;
grid on;
subplot(122)
DrawRobotImp(DH, 1, o_r, 1, o_d, 1, o_e(:,N), 1);
view([-30 35]);
title('End position');
grid on;
%% >>