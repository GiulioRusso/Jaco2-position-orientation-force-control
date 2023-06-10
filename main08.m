% MAIN07.M
%
% esercizio di sistemi robotici: controllo del moto
% PD + compensazione di gravita' per il Kinova Jaco2 nello spazio operativo

% pulisce spazio di lavoro
clear
close all

FLAG = 1;
% 1 legge classica
% 2 classica senza smorzamento nel nullo
% 3 compensando tutto
% 4 con pseudoinversa
if (FLAG==1) || (FLAG==2) || (FLAG==3)
    Kp = 2*diag(2*[20*ones(3,1); 7*ones(3,1)]);
    Kd = diag([7*ones(3,1); 1*ones(3,1)]);
    Kdq = diag([.4 .4 .4 .4 .4 .4 .1])
else
    % Questi sono i guadagni nel caso di una pseudoinversa (i casi
    % precedenti funzionano tutti con la trasposta, ed è importante
    % cambiare i guadagni poichè numericamente trasposta e inversa si
    % comportano diversamente: la trasposta lascia J al numeratore,
    % l'inversa mette J al denominatore, pertanto passando da inversa a
    % trasposta è necessario ritarare i guadagni)
    Kp = 0.1*diag(2*[20*ones(3,1); 7*ones(3,1)]);
    Kd = 0.05*diag([10*ones(3,1); 1*ones(3,1)]);
    Kdq = diag([.4 .4 .4 .4 .4 .4 .1]);
end

a     = zeros(7,1);
alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';
d     = [0.2755 0 -0.410 -0.0098 -0.3072 0 0.25]';
theta = [pi/4 pi/3 -pi/8 pi/3 -pi/12 pi/12 0]';
DH = [a alpha d theta];

% inizialite simulation variables
tf = 10;                     % tempo finale di simulazione
T_s = 1e-3;                 % passo di campionamento
n = 7;                      % 7 gradi di libertà
t = 0:T_s:tf;               % asse dei tempi
N = length(t);              % numero punti della simulazione
xd  = zeros(7,N);           % posa desiderata
x   = zeros(7,N);           % posa per ogni istante di simulazione
q   = zeros(n,N);           % posizione giunti
q(:,1) = [77  -17 0  43 -94 77 71]'/180*pi; % approximated home configuration
dq  = zeros(n,N);           % vettore velocità
ddq = zeros(n,N);           % vettore accelerazione
tau = zeros(n,N);           % vettore coppie
xtilde  = zeros(6,N);       % errore di posa

dx_c = [0.05 0.05 0.05];    % velocità di crociera

for i=1:N
    %% direct kinematics
    % Posizione e orientamento attuali
    % Estraggo la cinematica diretta con la configurazione corrente
    T = DirectKinematics([DH(:,1:3) q(:,i)]);
    % Estraggo la posizione corrente dell'e.e.
    x(1:3,i) = T(1:3,4,n);
    % Converto la matrice di rotazione in quaternione
    x(4:7,i) = Rot2Quat(T(1:3,1:3,n));
    
    %% desired end effector position
    % Estraggo dal profilo trapezoidale la posizione desiderata per seguire
    % il profilo. La posa finale è pari a quella iniziale variandola di 0.2
    % lungo x e y
    [xd(1:3,i),~,~] = trapezoidal(x(1:3,1),x(1:3,1)+[0.2 0.2 0]',dx_c,5,t(i));
    % Per l'orientamento, l'orientamento desiderato all'istante i-esimo è
    % uguale a quello all'istante 1 (cioè durante il movimento,
    % l'orientamento dell'e.e. rimane costante)
    xd(4:7,i) = x(4:7,1);
    
    %% errors
    % Errore di posizione = desiderato - corrente
    xtilde(1:3,i) = xd(1:3,i) - x(1:3,i);
    % Errore di orientamento = errore quaternione da QuatError 
    xtilde(4:6,i) = QuatError(xd(4:7,i),x(4:7,i));
    
    %% Geometric Jacobian
    % Calcolo lo Jacobiano geometrico con la configurazione corrente
    J = Jacobian([DH(:,1:3) q(:,i)]);
    
    %% control
    if FLAG==1 % 1 legge classica
        % In teoria noi dovremmo implementare slide 79 robotics10.pdf:
        %
        % u = g(q) + Ja(q)'*K_P*xtilde - Ja(q)'*K_D*Ja(q)*dq
        %
        % Dovremmo usare lo Jacobiano analitico, e inoltre questa versione va
        % bene se il manipolatore NON è ridondante.
        % Lo Jaco2 è ridondante, perciò questa legge di controllo non va bene.
        % Vediamo che slide 84 robotics10.pdf esiste una versione che,
        % retroazionando l'errore di orientamento con quaternione mi permette
        % di usare lo Jacobiano geometrico anzichè quello analitico, e che
        % inoltre ne esiste una versione per robot ridondante con trajectory
        % tracking
        %
        tau(:,i) = J'*Kp*xtilde(:,i) - J'*Kd*J*dq(:,i) + jaco2_g(q(:,i)) - Kdq*dq(:,i);
    elseif FLAG==2 % 2 classica senza smorzamento nel nullo
        % Levo lo smorzamento dato dal Kdq introdotto apposta per il
        % manipolatore ridondante. slide 79 robotics10.pdf:
        tau(:,i) = J'*Kp*xtilde(:,i) - J'*Kd*J*dq(:,i) + jaco2_g(q(:,i));
        
        % Gli errori di posizione e orientamento vanno a zero (anche se
        % potrebbero essere più puliti), ma osservando la velocità ai
        % giunti questa alla fine non è zero. Ma l'e.e. ha portato errore
        % nullo quindi alla posa desiderata ci è arrivato, perchè le
        % velocità non sono zero? Perchè senza quel termine di smorzamento
        % noi osserviamo una deriva nello spazio nullo: il robot continua a
        % muoversi nello spazio nullo pur essendo arrivato dove volevamo
    elseif FLAG==3 % 3 compensando tutto
        % Conoscendo il modello matematico compenso tutto, anche i termini
        % di Coriolis e Centrifughi. Utilizzando jaco2_n al posto di
        % jaco2_g compensiamo tutto: Gravità, Coriolis e Centrifughi (prima
        % compensavo solo la gravità, ora compenso tutto)
        % slide 85 robotics10.pdf:
        tau(:,i) = J'*Kp*xtilde(:,i) - J'*Kd*J*dq(:,i) + jaco2_n(q(:,i), dq(:,i));
        
        % Questi risultati come in main07template.m comunque mi portano dei
        % problemi di accoppiamento, ma la situazione è migliore poichè
        % "aiuto" il controllo compensando tutto il modello
        
        % In generale un robot leggero come lo Jaco che si muove a bassa
        % velocità avrà un termine di C(q,dq)*dq molto basso, perciò è
        % probabile che l'errore sarà anche basso
    else % 4 con pseudoinversa
        % Se so che J trasposto è una proiezione dal cartesiano ai giunti
        % tramite la dualità cinetostacia, posso pensare intuitivamente di
        % usare la pseudoinversa per fare questa proiezione
        tau(:,i) = pinv(J)*Kp*xtilde(:,i) - pinv(J)*Kd*J*dq(:,i) + jaco2_n(q(:,i), dq(:,i)) - Kdq*dq(:,i);
        
        % Cosa posso pensare se per un giunto vedo una accellerazione
        % variabile ma le relative coppie sono basse? Se la coppia è bassa
        % allora non dovrebbe accellerare no? Se succede signfica che
        % l'accellerazione che vediamo non è data dalla coppia ma
        % dall'interazione. Potrei diminuirla agendo sullo smorzamento
    end
    
    %% direct dynamics
    ddq(:,i) = jaco2_ddq(tau(:,i),q(:,i),dq(:,i));
    
    %% integration
    if i<N
        dq(:,i+1) = dq(:,i) + T_s*ddq(:,i);
        q(:,i+1)  = q(:,i) + T_s*dq(:,i);
    end
    
    clc
    fprintf('Controllo del moto\n')
    fprintf('Percentuale di completamento %.0f %% \n',(i/N)*100)
end
fprintf('\n');

figure
subplot(211)
plot(t,q), grid
xlabel('t [s]'),ylabel('q [rad]'), title('joints pos')
legend('1','2','3','4','5','6','7')
subplot(212)
plot(t,dq),grid
xlabel('t [s]'),ylabel('dq [rad/s]'), title('joints vel')
legend('1','2','3','4','5','6','7')

% Il profilo di dq non è trapezoidale poichè lo è nello spazio operativo.
% Dallo spazio operativo proietto nello spazio giunti tramite la J'

figure
subplot(211)
plot(t,xtilde(1:3,:)),grid
xlabel('t [s]'),ylabel('e.e. pos err [m]'), title('pos err')
subplot(212)
plot(t,xtilde(4:6,:)),grid
xlabel('t [s]'),ylabel('e.e. or err [-]'), title('or err')

figure
subplot(211)
plot(t,ddq),grid
xlabel('t [s]'),ylabel('ddq [rad/s^2]'), title('joints acc')
legend('1','2','3','4','5','6','7')
subplot(212)
plot(t,tau),grid
xlabel('t [s]'),ylabel('tau [Nm]'), title('joints tau')
legend('1','2','3','4','5','6','7')

figure
hold on
DH(:,4) = q(:,1);
DrawRobot(DH);
DH(:,4) = q(:,N);
DrawRobot(DH);