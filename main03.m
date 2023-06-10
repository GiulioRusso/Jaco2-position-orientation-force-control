% MAIN03.M
%
% Robotic systems practice: trapezoidal velocity profile
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

% clean workspace
clear
close all
clc

% inizialite variables
q_i = [0 -20 30]'/180*pi;   % posizione iniziale dei giunti [gradi]*pi/180
q_f = [10 -40 60]'/180*pi;  % posizione finale dei giunti [gradi]*pi/180
tf = 2;                     % tempo finale

% La velocità di crociera del profilo trapezioidale è la velocità massima
% della traiettoria ed è soggetta ad un limite fisico (spazio/tempo)
%
% abs(q_f-q_i)/tf < dq_c < 2*abs(q_f-q_i)/tf
%
% dq_c (velocità di crociera) deve essere compresa tra il minimo di
% velocità possibile per andare da q_f a q_i (tutte le velocità sotto
% questo lower bound ci fanno andare da q_i a q_f con un tf maggiore di
% quello specificato) e il massimo pari al doppio della velocità minima
% (profilo triangolare di velocità)
dq_c = [1.5 1.6 1.8]'.*abs(q_f-q_i)/tf
Ts = 1e-3;                  % passo di campionamento

n = size(q_i,1);            % numero di giunti
t = 0:Ts:1.5*tf;            % asse dei tempi
N = length(t);              % numero dei punti della simulazione

% Definiamo le matrici di posizione, velocità e accelerazione. Queste hanno
% dim nxN dove:
% - ogni colonna (di dimensione n) contiene tutte le informazioni della 
% posizione(q)/velocità(dq)/accelerazione(ddq) per tutti i giunti. Avendo N
% colonne avrò queste informazioni per ogni punto della simulazione
% - ogni riga (di dimensione N) contiene tutte le informazioni della
% posizione(q)/velocità(dq)/accelerazione(ddq) del singolo giunto i-esimo
% ad ogni istante della simulazione
%
% q(j,i) = posizione del giunto j all'istante i
% dq(j,i) = velocità del giunto j all'istante i
% ddq(j,i) = accelerazione del giunto j all'istante i
q = zeros(n,N);
dq = zeros(n,N);
ddq = zeros(n,N);

% Applico il profilo trapezioidale per ogni istante della simulazione
for i=1:N
    % Computo posizione(q)/velocità(dq)/accelerazione(ddq) di tutti gli n
    % giunti all'istante i-esimo (memorizzo il tutto nella colonna i-esima
    % di q, dq e ddq)
    [q(:,i),dq(:,i),ddq(:,i)] = trapezoidal(q_i,q_f,dq_c,tf,t(i));
end

% plot
figure
subplot(311)
plot(t,q,'.'),grid
title('Position')
ylabel('pos')
subplot(312)
plot(t,dq),grid
title('Velocity')
ylabel('vel')
subplot(313)
plot(t,ddq,'-.'),grid
title('Acceleration')
ylabel('acc')
