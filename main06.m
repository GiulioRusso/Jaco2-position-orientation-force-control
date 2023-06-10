% MAIN06.M
%
% Robotic systems practice: differential kinematic inversion
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

% clean workspace
clear
close all
clc

% DH table initialization
fprintf('\n jaco2 7 links \n')
% parameters
% slide 65 robotics02.pdf
a     = zeros(7,1);
alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';
d     = [0.2755 0 -0.410 -0.0098 -0.3072 0 0.25]';
theta = [pi/4 pi/3 -pi/8 pi/3 -pi/12 pi/12 0]'; % 
DH = [a alpha d theta];

% Scegliere l'algoritmo 'transpose'/'inverse'
algorithm='transpose';%'inverse';

% Ha senso dividere i guadagni per posizione e orientamento. Ha meno senso
% pesare diversamente per esempio in direzioni come x e y
if strcmp(algorithm,'inverse')
    K = diag([10*[1 1 1], 5*[1 1 1]]);
    fprintf('\n algorithm: inverse of the jacobian \n')
else
    K = diag([50*[1 1 1], 30*[1 1 1]]);
    fprintf('\n algorithm: transpose of the jacobian \n')
end

% inizialite simulation variables
tf = 2;
Ts = 1e-3;
n = 7;                          % 7 gradi di libertà
t = 0:Ts:tf;
N = length(t);
xd  = zeros(3,N);
x   = zeros(3,N);
q   = zeros(n,N);
q(:,1) = [45 -50 -15 90 150 25 -90]'/180*pi;
% Posizione di partenza in singolarità: lo so poichè il manipolatore tutto
% steso è singolare, ma da cosa me ne accorgo?
% 1- il numero di condizione parte da 10^17 (elevato) infatti sono partito
% da una posizione di singolarità
% 2- le velocità partono subito da valori molto alti per poi scendere. Qui
% non c'entra il discorso sul profilo di velocità, il valore è
% significativamente alto rispetto al valore a cui tendono dopo qualche
% istante. Non si blocca nulla poichè usiamo un software numerico
% 3- le posizioni fanno letteralmente un salto da un valore ad un altro
%
% Usando pinv_damped.m nell'algoritmo della inversione, le discontinuità
% sono mitigate
q(:,1) = [0 0 0 180 0 0 0]'/180*pi;


% Posizione di partenza verticale
%q(:,1) = [pi/2 0 0 pi 0 0 0]';
dq  = zeros(n,N);
quat = zeros(4,N);
quat_d = zeros(4,N);
error_pos = zeros(3,N);
error_quat = zeros(3,N);
error = zeros(6,N);
condJ = zeros(1,N);

for i=1:N
    %% generate desired e.e. value
    % desired task trajectory
    xd(:,i) = [0.2 0.15 1]';
    quat_d(:,i) = [0 0 0 1]';
    
    %% compute current e.e. value
    % direct kinematics
    DH(:,4) = q(:,i);
    T = DirectKinematics(DH);
    x(:,i) = T(1:3,4,n);                   % posizione
    quat(:,i) = Rot2Quat(T(1:3,1:3,n));    % quaternione
    
    %% compute controller's output
    % Jacobian
    J = Jacobian(DH); % questa volta mi interessano tutte le componenti
    condJ(i) = cond(J);
    % Inverse kinematics algorithm
    error_pos(:,i) = xd(:,i) - x(:,i);
    error_quat(:,i) = QuatError(quat_d(:,i),quat(:,i));
    error(:,i) = [error_pos(:,i);error_quat(:,i)];
    
    % Usando i quaternioni possiamo usare lo Jacobiano geometrico di base
    % slide 2 robotics09.pdf consideriamo dxd = 0
    % slide 127 robotics03.pdf algoritmo cinematico inverso
    if strcmp(algorithm,'transpose')
        dq(:,i) = J'*K*error(:,i);
    else
        dq(:,i) = pinv_damped(J,eye(n))*K*error(:,i);
        %dq(:,i) = pinv(J)*K*error(:,i);
    end
    if i<N
        q(:,i+1) = q(:,i) + Ts*dq(:,i);
    end
end


figure
subplot(231)
plot(t,q),grid
ylabel('q [rad]'), title('joints pos')
xlabel('time [s]')
subplot(234)
plot(t,dq),grid
ylabel('dq [rad/s]'), title('joints vel')
xlabel('time [s]')
subplot(232)
plot(t,error(1:3,:)),grid
ylabel('p err [m]'), title('cartesian pos err')
xlabel('time [s]')
subplot(235)
plot(t,error(4:6,:)),grid, title('quaternion or err')
ylabel('o err [-]')
xlabel('time [s]')
subplot(233)
plot(t,condJ),grid
ylabel('cond J'), title('Jacobian condition number')
xlabel('time [s]')

figure
hold on
DH(:,4) = q(:,1);
DrawRobot(DH);
DH(:,4) = q(:,N);
DrawRobot(DH);

% Qui i guadagni hanno lo stesso significato degli autovalori in un sistema
% dinamico: più sono alti, più l'errore (la dinamica) è veloce. In questo
% caso sappiamo che le equazioni sono tali da portar l'errore a zero
% In generale vige un trade-off tra guadagno e tempo finale necessario per
% portare a termine il task richiesto. Se diminuisco K impiegherò più tempo
% per arrivare alla posa desiderata, se lo aumento ci arrivo più
% velocemente 

