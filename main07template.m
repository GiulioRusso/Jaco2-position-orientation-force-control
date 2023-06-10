% MAIN07.M
%
% esercizio di sistemi robotici: controllo del moto
% PD + compensazione di gravita' per il Kinova Jaco2 nello spazio giunti

% pulisce spazio di lavoro
clear
close all

%paramError = 0.5;
%PerturbedParameters = DynamicParameters * (1-paramError);

%Kp = diag([1 200 0.5 80 90 30 3]);
%Kd = diag([100 200 23 21 2 1 13]);



% inizializza variabili simulazione
n = 7;              % numero giunto
t_f = 3;            % tempo finale simulazione
t_f_moto= 2;        % tempo per finire il moto del robot
% Sono interessato ad avere un tempo finale maggiore del tempo nel quale il
% robot deve completare il movimento, poichè così posso vedere il
% transitorio dei giunti. Se dopo t_f_moto vedo che i giunti ancora hanno
% velocità non nulle vuol dire che stanno oscillando e quindi che i
% guadagni sono tarati male
T_s = 1e-3;         % passo di campionamento
t = 0:T_s:t_f;      % asse dei tempi
N = length(t);      % numero di punti della simulazione

% Vettore delle posizioni per ogni punto della simulazione
q   = zeros(n,N);
% Posa di partenza del robot
q(:,1) = [77  -17 0  43 -94 77 71]'/180*pi; % approximated home configuration
% Posa finalde del robot da raggiungere
q_final = q(:,1) + pi/2;%*[0 0 0 0 0 0 1]';
% Usando il vettore *[0 0 0 0 0 0 1]' possiamo tarare i guadagni uno alla
% volta spostando l'1 per migliorare il comportamento dei giunti, oppure
% anche coppie insieme (*[0 0 0 0 0 1 1]'). In generale qui vediamo che il
% controllo non tiene conto dell'accoppiamento dei giunti. Muovendo un solo
% giunto alla volta posso arrivare ad un punto in cui il movimento che fa è
% perfetto ed è proprio quello che voglio, ma appena faccio muovere un
% altro giunto assieme a questo, allora vedo che le prestazioni del primo
% calano e che in generale posso anche avere dei problemi

% Vettori di velocità e accelerazione per ogni punto della simulazione
dq  = zeros(n,N);
ddq = zeros(n,N);

% Vettori di posizione e velocità desiderata data dal profilo trapezioidale
q_d  = zeros(n,N);
dq_d  = zeros(n,N);
% Velocità di crociera per ogni giunto
dq_c = [1 1 1 1 1 1 1]';
% Se dessi delle velocità di crociera diverse tra i giunti, osservo che il
% trapezio del profilo di velocità potrebbe cominciare in punti diversi
% dell'asse dei tempi. A causa del fatto che il controllo non tiene conto
% dell'accoppiamento tra i giunti, è possibile osservare delle
% discontinuità sull'accelerazione, date dal fatto che in base a dove mi
% trovo sul profilo di velocità con un giunto, l'accoppiamento con gli
% altri giunti va e viene: può capitare che appena un giunto arriva a
% velocità di crociera, questo crea una discontinuità sui profili di
% velocità e accelerazione degli altri giunti. Ciò può accadere per esempio
% se aumento troppo i guadagni (se devo smorzare posso pensare di aumentare
% K_D, ma se questo mi provoca un accoppiamento, allora posso pensare di
% mantenere il vecchio K_D e diminuire invece il K_P, che so che più è alto
% e più aumenta le oscillazioni). In generale guadagni troppo alti possono
% darmi problemi di discretizzazione (Matlab darà qualche errore)

% Sempre guardare dai grafici che i picchi di velocità e accelerazione 
% siano entro i limiti stabiliti del robot

% Vettore delle coppie ai giunti
tau = zeros(n,N);
% Vettore di errore di posizione e velocità
q_tilde  = zeros(n,N);
dq_tilde = zeros(n,N);

% Guadagni KP e KD
K_P = diag([200 60 50 50 50 50 0.5]);
K_D = diag([4 12 3 3 1 1 0.05]);
% Quando si cambiano i guadagni, conservare sempre i valori precedenti

% loop
for i=1:N
    
    % t(i): Possiedo già la posizione corrente del robot (i=1 coincide con
    % la configurazione iniziale)
    
    %% desired joint position
    % Calcolo la posizione e velcità da inseguire tramite il profilo 
    % trapezoidale. Questa volta assegno un profilo di velocità da me 
    % scelto. Richiede: 
    % - posizione iniziale: q(:,1)
    % - poszione finale: q_final
    % - velocità di crociera: dq_c
    % - tempo finale di moto: tempo in cui completare il movimento (diverso
    % dal tempo finale di simulazione)
    % - istante di tempo: t(i)
    %
    % Resituisce:
    % - posizione all'istante t(i): q_d(:,i) 
    % - velocità all'istante t(i): dq_d(:,i) 
    [q_d(:,i), dq_d(:,i), ~] = trapezoidal(q(:,1), q_final,dq_c, t_f_moto, t(i));
    
    %% errors
    % Calcolo l'errore tra posizione corrente e quella da inseguire
    % ottenuta tramite il profilo trapezoidale. L'errore è da calcolare
    % nello spazio giunti, per cui è semplicemente la grandezza desiderata
    % meno quella vera
    q_tilde(:,i) = q_d(:,i) - q(:,i);
    dq_tilde(:,i) = dq_d(:,i) - dq(:,i);
    
    %% control
    % Calcolo le coppie da dare ai giunti per portare l'errore a zero
    % slide 49 robotics10.pdf
    %
    % Caso con dqd = 0
    % u = g(q) + K_P * q_tilde - K_D * dq
    %
    % Ma noi abbiamo una velocità desiderata (imposta dal profilo
    % trapezoidale), allora dqd è diverso da 0. L'equazione diventa:
    % u = g(q) + K_P * q_tilde - K_D * dq (+ K_D * dqd)
    %   = g(q) + K_P * q_tilde + K_D * dq_tilde
    
    tau(:,i) = K_P*q_tilde(:,i) + K_D*dq_tilde(:,i) + jaco2_g(q(:,i));
    % Lavodando sullo Jaco2, per la compensazione della gravità usiamo la
    % funzione apposita
    
    % Per capire cosa succede se sbagliamo la compensazione aggiungiamo una
    % costande davanti allo jaco2_g:
    %
    % tau(:,i) = K_P*q_tilde(:,i) + K_D*dq_tilde(:,i) + .9*jaco2_g(q(:,i));
    %
    % Nella realtà non si sbaglia di un valore proporzionale a questo, ma 
    % si sbagliano valori come masse, momenti primi di interzia ecc... 
    % questo però, pur non essendo un comportamento fisico corretto, 
    % comunque ci fa capire cosa succede se la compensazione non è 
    % perfetta)
    %
    % Cosa succede? L'errore di posizione NON va a zero. La legge di
    % controllo è senza azione integrale e con questa costante modificata
    % compenserà anche male la gravità, e quindi il robot si ferma in una
    % posizione che non è quella desiderata. In generale cambia anche in
    % base alla configurazione, per cui l'errore che commetto è poco
    % prevedibile
    
    %% direct dynamics
    % Usando:
    % - il controllo: tau(:,i)
    % - posizione: q(:,i)
    % - velocità: dq(:,i)
    % Calcolo:
    % - l'accelerazione: ddq(:,i)
    % tramite la funzione di Dinamica diretta dello Jaco2
    ddq(:,i) = jaco2_ddq(tau(:,i),q(:,i),dq(:,i));
    
    %% integration
    % Per avere posizione e velocità al prossimo passo
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
plot(t,q),grid
xlabel('t [s]'),ylabel('q [rad]'), title('joints pos')
legend('1','2','3','4','5','6','7')
subplot(212)
plot(t,dq),grid
xlabel('t [s]'),ylabel('dq [rad/s]'), title('joints vel')
legend('1','2','3','4','5','6','7')

figure
subplot(211)
plot(t,q_tilde),grid
xlabel('t [s]'),ylabel('err q [rad]'), title('pos err')
legend('1','2','3','4','5','6','7')
subplot(212)
plot(t,dq_tilde),grid
xlabel('t [s]'),ylabel('err dq [rad/s]'), title('vel err')
legend('1','2','3','4','5','6','7')

figure
subplot(211)
plot(t,ddq),grid
xlabel('t [s]'),ylabel('ddq [rad/s^2]'), title('joints acc')
legend('1','2','3','4','5','6','7')
subplot(212)
plot(t,tau),grid
xlabel('t [s]'),ylabel('tau [Nm]'), title('joints tau')
legend('1','2','3','4','5','6','7')

