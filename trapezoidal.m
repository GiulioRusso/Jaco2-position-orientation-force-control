function [y,dy,ddy] = trapezoidal(y_i,y_f,dy_c,tf,t)
%
% trapezoidal velocity profile at instant t
% 
%   [y,dy,ddy] = trapezoidal(y_i,y_f,dy_c,t_f,t) 
%
%   input:
%       y_i     dim nx1     initial value
%       y_f     dim nx1     final value
%       dy_c    dim nx1     cruise velocity
%       tf      dim 1       final time
%       t       dim 1       time
%   output:
%       y       dim nx1     position at t
%       dy      dim nx1     velocity at t
%       ddy     dim nx1     acceleration at t
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

% input as columns
[n,m] = size(y_i);
if m>n
    y_i = y_i';
    y_f = y_f';
    dy_c = dy_c';
end

n = size(y_i,1);        % ricalcolo il numero di giunti

% Inizializzo i vettori di posizione(y), velocità(dy) e accelerazione(ddy)
y   = zeros(n,1);
dy  = zeros(n,1);
ddy = zeros(n,1);

% repeat for each dimension (1/2/3)
for i=1:n
    
    % Calcolo quanto manca dalla posizione corrente per arrivare alla
    % posizione finale
    %
    % delta = posizione finale - posizione corrente
    %
    delta = y_f(i) - y_i(i);
    
    % Aggiungo al valore di velocità di crociera il segno. La velocità di
    % crociera specificata è positiva, ma in base al segno del delta della
    % posizione può essere positiva o negativa, pertanto questa viene
    % riassegnata nella funzione selezionando il segno del delta * il
    % valore assoluto della velocità di crociera. Questo valore coincide
    % con il tratto di linea dritta del profilo di velocità
    dy_c(i) = sign(delta)*abs(dy_c(i));
    
    % Se delta = 0, cioè il punto iniziale e finale specificati dall'utente
    % coincidono, non devo spostarmi, quindi:
    % 
    % y(i) =  alla posizione iniziale
    % dy(i) = 0 non ho velocità
    % ddy(i) = 0 non accellero
    %
    if (delta==0)
        y(i)   = y_i(i);
        dy(i)  = 0;
        ddy(i) = 0;
        
    % Se delta NON è 0, i giunti devono spostarsi per raggiungere la
    % posizione finale richiesta
    else
        % constraint verification for joint i
        % Verifico il constraint per avere una velocità di crociera
        % plausibile come spiegato nel main03.m.
        dy_r = abs(delta/tf);
        err = (abs(dy_c(i)) <= dy_r)|(abs(dy_c(i)) > 2*dy_r);
        
        % Se err = 1 la velocità di crociera scelta NON è ammissibile.
        % Restituisco un errore
        if (err==1)
            % Per evitare di far bloccare il programma potremmo assegnare
            % forzatamente la velocità di crocciera al valore limite
            % dy_c(i) = (2*dy_r);
            fprintf('assigned cruise velocity: %.2f\n',dy_c(i))
            fprintf('limits: [%.2f--%.2f]\n',dy_r,2*dy_r)
            error('error in cruise velocity');
        
        % Altrimenti procedo con l'identificazione delle variabili
        else
            % slide 24 robotics05.pdf
            % dq = q/t                  (v = s/t)
            % ddq = dq/t                (a = v/t)
            % q = q0 + 0.5 * ddq * t^2  (s = s0 + 0.5*a*t^2)
            
            % evaluates t_c
            % t_c = (qi-qf+dqc*tf)/dqc -> vedo che delta = qf-qi
            %     = (-delta+dqc*tf)/dqc -> eseguo la divisione per dqc
            %     = -delta/dqc + tf -> scambio i termini
            %     = tf - delta/dy_c(i)
            t_c = tf - delta/dy_c(i);
            
            % evaluates ddy_c
            % ddqc = (dqc^2)/(qi-qf+dqc*tf)
            %      = (dqc^2)/(-delta+dqc*tf) -> porto un dqc dal dqc^2
            %      sotto il (-delta+dqc*tf)
            %      = dqc/((-delta+dqc*tf)/dqc) -> il denominatore è tc
            %      = dqc/tc
            ddy_c = dy_c(i)/t_c;
            % if on the time slots
            if (t<=t_c) % Fase di accelerazione
                y(i)   = y_i(i) + 0.5*ddy_c*t^2; % moto accelerato: s0 + 1/2*a*t
                dy(i)  = ddy_c*t; % derivata di y(i) rispetto a t
                ddy(i) = ddy_c; % derivata di dy(i) rispetto a t
            elseif (t<=(tf-t_c)) % Fase di crociera
                y(i)   = y_i(i) + dy_c(i)*(t-0.5*t_c); % moto uniforme: s0 + v*t
                dy(i)  = dy_c(i); % derivata di y(i) rispetto a t
                ddy(i) = 0; % derivata di dy(i) rispetto a t
            elseif (t<=tf) % Fase di decellerazione
                y(i)   = y_f(i) - 0.5*ddy_c*(t-tf)^2; % moto decelerato: s0 - 1/2*a*t
                dy(i)  = ddy_c*(tf-t); % derivata di y(i) rispetto a t
                ddy(i) = -ddy_c; % derivata di dy(i) rispetto a t
            else % t>tf
                y(i)   = y_f(i); % la posizione rimane uguale a quella finale (posizione = posizione finale)
                dy(i)  = 0; % i giunti sono fermi (velocità = 0)
                ddy(i) = 0; % e non accelerano (accelerazione = 0)
            end
        end
    end
end
