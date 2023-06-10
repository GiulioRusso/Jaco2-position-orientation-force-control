% MAIN05.M
%
% Robotic systems practice: differential kinematic inversion
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

% clean workspace
clear
close all
clc

% inizialite robot variables
fprintf('\n planar 3 links \n')
% parameters
a     = [1 .4 .2]';
alpha = zeros(3,1);
d     = zeros(3,1);
theta = [45 -10 -10]'/180*pi;
DH = [a alpha d theta];

% Scegliere l'algoritmo 'transpose'/'inverse'
algorithm='transpose';%'inverse';

if strcmp(algorithm,'inverse')
    K = 1*diag([10 10 10 10 10]);
    fprintf('\n algorithm: inverse of the jacobian \n')
else
    K = 10*diag([9 9 12 12 12]);
    fprintf('\n algorithm: transpose of the jacobian \n')
end


% inizialite simulation variables
tf = 1;                     % tempo finale
Ts = 1e-3;                  % passo di campionamento
r = 2;                      % dim. task solo xy
n = size(DH,1);             % numero giunti
t = 0:Ts:tf;                % asse dei tempi
N = length(t);              % numero di punti della simulazione
xd  = zeros(r,N);           % vettore posa desiderata coordinate [x y orientamento]
x   = zeros(r,N);           % vettore posa per ogni punto della simulazione
q   = zeros(n,N);           % Vettore delle posizioni (theta) dei giunti
q(:,1) = DH(:,4);           % Inizializzo la posizione dei giunti ai theta della DH
dq  = zeros(n,N);           % Vettore delle velocità dei giunti per ogni punto della simulazione


% Vettore dei quaternioni per ogni punto della simulazione
quat   = zeros(4,N);
quat_d = zeros(4,N); % quaternione desiderato

% Errore di posizione
error_pos  = zeros(r,N);
% Errore del quaternione
error_quat = zeros(3,N);
% Errore totale
error      = zeros(5,N);

% Numero di condizione dell Jacobiano per vedere quanto sono vicino alla
% singolarià (divisione per zero)
condJ = zeros(1,N);

for i=1:N
    %% generate desired e.e. value
    % desired task trajectory
    
    % Posa x y desiderata
    xd(:,i) = [.9 1]'; %example of nice pose
    
    % xd(:,i) = [2 2]';
    % Esempio di singolarità: la posizione dell'e.e. non è raggiungibile
    % per come sono lunghi i giunti
    
    % Quaternione desiderato
    quat_d(:,i) = [0 0 0 1]';
    
    %% compute current e.e. value
    % direct kinematics
    DH(:,4) = q(:,i);
    T = DirectKinematics(DH);
    
    % Prendo la posizione x e y dal vettore posizione dalla trasformazione
    % omogenea
    x(1:2,i) = T(1:2,4,n);
    % Calcolo il quaternione a partire dalla matrice di rotazione
    quat(:,i) = Rot2Quat(T(1:3,1:3,n));
    
    %% compute controller's output
    % Jacobian
    J = Jacobian(DH);
    % Cosa mi interessa del planare a tre bacci? posizione x e y
    % sicuramente si (infatti la posa xd specificata ha due valori, proprio
    % per x e y). L'orientamento invece è calcolato tramite i quaternioni
    % (in main04.m l'orientamento era scalare e pari alla somma dei theta
    % dei giunti del robot) e otteniamo un errore 3x1. Pertanto dalla
    % posizione solamente le due componenti x e y ci interessano, per
    % l'orientamento ci interessano tutte e 3, perciò lo Jacobiano prenderà
    % le prime due componenti (x e y) e le ultime 3 (dell'orientamento)
    J = [J(1:2,:); J(4:6,:)];
    
    % Calcolo il numero di condizione di J
    condJ(i) = cond(J);
    
    % Inverse kinematics algorithm
    % Errore di posizione
    error_pos(:,i)  = xd(:,i) - x(:,i);
    % Errore di orientamento con Quaternioni
    error_quat(:,i) = QuatError(quat_d(:,i),quat(:,i));
    % Errore finale (posizione e orientamento)
    error(:,i)      = [error_pos(:,i);error_quat(:,i)];
    
    % L'errore del quaternione è definito su 3 componenti, ma alla fine 2
    % sono nulle poichè il movimento del robot è sul piano xy. La stessa
    % cosa accade per le componenti dello Jacobiano per la stessa ragione.
    % Siccome però l'errore è 3x1 ce lo portiamo dietro uguale
    
    % pag. 132 del libro: lo Jacobiano geometrico e analitico sono uguali
    % per un planare a tre bacci. Ma usando i quaternioni possiamo usare lo
    % Jacobiano geometrico di base
    % slide 2 robotics09.pdf consideriamo dxd = 0
    if strcmp(algorithm,'transpose')
        % slide 94 robotics03.pdf
        dq(:,i) = J'*K*error(:,i);
    else
        % slide 98 robotics03.pdf
        dq(:,i) = pinv(J)*K*error(:,i);
    end
    %% integration
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
plot(t,error(1:2,:)),grid
ylabel('p err [m]'), title('cartesian pos err')
xlabel('time [s]')
subplot(235)
plot(t,error(3:5,:)),grid, title('quaternion or err')
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

% Se ho una singolarità nello spazio di lavoro, vedo dei picchi negli
% errori, poichè con alte velocità date dalla singolarità ho scostamenti
% dell'ordine dei radianti (che corrispondono a diversi giri nella 
% circonferenza) e alla iterazione successuva potrò trovarm in ogni punto 
% della circonferenza.
