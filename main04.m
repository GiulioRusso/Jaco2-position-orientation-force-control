% MAIN04.M
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
% DH parameters
a     = [1 .4 .2]';
alpha = zeros(3,1);
d     = zeros(3,1);
theta = [45 -10 -10]'/180*pi;
DH = [a alpha d theta];

% Scegliere l'algoritmo 'transpose'/'inverse'
algorithm='transpose';%'inverse';

% Il guadagno K è identificato come [Kp Kp Ko]: prime due componenti per la
% posizione, l'ultima per l'orientamento. 3x3 diagonale
if strcmp(algorithm,'inverse')
    K = diag([10 10 10]);
    fprintf('\n algorithm: inverse of the jacobian')
else
    K = diag(5*[8 8 10]);
    fprintf('\n algorithm: transpose of the jacobian')
end
fprintf('\n')

% inizialite simulation variables
tf = 1;             % tempo finale
Ts = 1e-3;          % passo di campionamento
r = 3;              % dim. task
n = size(DH,1);     % numero giunti
t = 0:Ts:tf;        % asse dei tempi
N = length(t);      % numero di punti della simulazione
xd  = zeros(r,N);   % vettore posa desiderata coordinate [x y z]
x   = zeros(r,N);   % vettore posa per ogni punto della simulazione

% Il vettore della posa 'x' ha dimensione pari a quella del task di
% interesse. Come da slide 2 robotics09.pdf il task è pari alla coordinata
% x dell'e.e., coordinata y dell'e.e., e l'orientamento è calcolato come la
% somma dei theta dei giunti (questo vale SOLO nel caso di un planare a 3
% bracci). Costruiamo 'x' come:
%      
% x =  [x e.e.  y e.e  orientamento e.e.]'
% 
% orientamento e.e. = theta giunto 1 + theta giunto 2 + theta giunto 3

% Vettore delle posizioni (theta) dei giunti per ogni punto della
% simulazione
q   = zeros(n,N);
q(:,1) = DH(:,4); % Inizializzo la posizione dei giunti ai theta della DH

% Vettore delle velocità dei giunti per ogni punto della simulazione
dq  = zeros(n,N);

% Numero di condizione dell Jacobiano per vedere quanto sono vicino alla
% singolarià (divisione per zero)
condJ = zeros(n,1);

% Computiamo posizione e velocità (e i relativi errori) per ogni istante
% della simulazione
for i=1:N
    %% generate desired e.e. value
    
    % desired task trajectory
    %
    % xd = [x e.e. y e.e. orientamento e.e.]
    %
    % e.g. xd = [1 1 1.54] (1.54 rad = 90 gradi)
    %               
    % 1         x (e.e.)
    %           |
    %           |
    % 0
    %  0        1 
    %
    % e.g. xd = [1 1 0.77] (0.77 rad = 45 gradi)
    %
    % 1         x (e.e.)
    %          /
    %         / 
    % 0
    %  0        1 
    %
    xd(:,i) = [1 1 1.54]';
    
    % xd(:,i) = [1.7 0 0]';
    % Singolarità cinematica: ho un robot con somma dei bracci (a) pari a
    % 1.6 e io richiedo l'e.e. in posizione x = 1.7 (indipenden temente da
    % y e l'orientamento, se il robot è lungo totalmente 1.6, a 1.7 non ci
    % arriva), ilbraccio del robot chiaramente non può arrivarci per motivi
    % fisici (è troppo corto). Vedremo che il numero di condizione 
    % esploderà
    
    %% compute current e.e. value
    
    % direct kinematics
    % Aggiorno la posizione (theta) dei giunti all'iterazione corrente
    % (per i = 1 sarà uguale al valore inizializzato q(:,1) = DH(:,4))
    % Con queste posizioni (theta) dei giunti computo la cinematica diretta
    DH(:,4) = q(:,i);
    T = DirectKinematics(DH);
    
    % La cinematica diretta mi restituisce: (posizione giunti = theta)
    % - Matrice di Rotazione 3x3 per passare dalla posizione dei giunti 'q' 
    % corrente (all'istante i-esimo) alla posizione dei giunti 'q' 
    % desiderata
    % - Vettore posizione in colonna 4, di cui prendo solo le componenti
    % x e y, n poichè v voglio la cinematica diretta fatta fino a tutti i
    % giunti
    
    % Posizione x, y presa dai primi due elementi del vettore posizione
    % (primi 3 elementi 4a colonna della matrice di trasformazione
    % omogenea ottenuta dalla cinematica diretta), ma di quale giunto?
    % Dell'e.e., allora scegliamo l'indice n (l'ultimo giunto)
    x(1:2,i) = T(1:2,4,n);
    
    % Orientamento = somma dei theta dei tre giunti
    x(3,i) = q(1,i) + q(2,i) + q(3,i); % only for 'planar_3link'
    
    %% compute controller's output
    
    % Jacobian
    J = Jacobian(DH);
    
    % Con la funzione Jacobian(DH) ottengo uno jacobiano 6x6, ma a me
    % interessa tutto? No, dalla dimensione del task richiesta interessa
    % solamente la posizione x, y e l'orientamento. Siccome ho un planare a
    % 3 bracci, osservando J vedo che delle 3 componenti dell'orientamento,
    % due righe sono nulle, pertanto prendo solamente quella diversa da
    % zero, cioè la 6a riga (se includessi righe nulle in J, questo non
    % sarebbe a rango pieno)
    J = [J(1:2,:); J(6,:)];
    
    % Calcolo il numero di condizione di J
    condJ(i) = cond(J);
    
    % Inverse kinematics algorithm
    % slide 2 robotics09.pdf consideriamo dxd = 0 (il profilo di velocità
    % non lo analizziamo, infatti vedremo che nel grafico delle velocità
    % dei giunti abbiamo dei "salti", cioè i grafici partono già da valori
    % di velocità diversi da 0. Questo è assurdo poichè un robot parte da
    % fermo con velocità nulle ai giunti. Quando aggiungeremo il profilo di
    % velocità trapezoidale questo cambierà)
    
    % pag. 132 del libro: lo Jacobiano geometrico e analitico possono
    % essere uguali solo nel caso in cui i gradi di libertà possono imporre
    % una rotazione dell'e.e. attorno ad un solo asse. Nel caso di un
    % planare a 3 bracci con giunti rotoidali questo è il caso: l'e.e. può
    % ruotare solo intorno all'asse z. Definendo l'orientamento come somma
    % dei theta dei 3 giunti, questo coincide con la definizione dello
    % Jacobiano geometrico. Pertanto qui utilizzeremo gli algoritmi di
    % inversione cinematica come visti nelle slide, ma anzichè creare una
    % funzione per lo Jacobiano analitico, utilizzeremo quella dello
    % Jacobiano geometrico poichè in questo caso specifico, coincidono.
    if strcmp(algorithm,'transpose')
        % slide 94 robotics03.pdf
        % e = xd - x
        % dq = J' * K * e
        %
        dq(:,i) = J' * K * (xd(:,i) - x(:,i));
    else % 'inverse'
        % slide 92 robotics03.pdf
        % e = xd - x
        % dq = pinv(J)(dxd + K * e) -> noi consideriamo dxd = 0, allora:
        %    = pinv(J) * K * e
        %
        dq(:,i) = pinv(J) * K * (xd(:,i) - x(:,i));
        % uso pinv (pseudoinversa) per il caso ridondante. Se non 
        % necessario, Matlab usa inv (inversa) come faremmo per il caso NON
        % ridondante (con pinv è più generale)
    end

    %% integration
    % il ciclo va da 1 a N, per i=N crea q(:,N+1), ma non ci interessa,
    % allora usiamo un if per fermarci prima
    if i<N
        q(:,i+1) = q(:,i) + Ts*dq(:,i);
    end
end


figure
hold on
DH(:,4) = q(:,1);
DrawRobot(DH);
DH(:,4) = q(:,N);
DrawRobot(DH);

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
plot(t,xd(1:2,:)-x(1:2,:)),grid
ylabel('p err [m]'), title('cartesian pos err')
xlabel('time [s]')
subplot(235)
plot(t,xd(3,:)-x(3,:)),grid
ylabel('o err [rad]'), title('cartesian or err')
xlabel('time [s]')
subplot(233)
plot(t,condJ),grid
ylabel('cond J'), title('Jacobian condition number')
xlabel('time [s]')

% Cosa fare se l'errore sta diminuendo ma non va a zero nel tf specificato?
% O aumento tf, oppure aumento il guadagno