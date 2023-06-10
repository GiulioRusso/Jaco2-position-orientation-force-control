function  T0 = DirectKinematics(DH)
%
% Homogeneous transformation matrices with respect to the arm base frame 
% T0 = DirectKinematics(DH)
%
% input:
%       DH     dim nx4     Denavit-Hartenberg table (a, alpha, d, theta)
%
% output:
%       T      dim 4x4xn   Homogeneous transformation matrices with respect to the arm base frame    
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

    n = size(DH,1); % numero di giunti
    T0 = zeros(4,4,n);
    T  = zeros(4,4,n);

    % Homogeneous transformation matrix between consecutive frames according to DH convention
    
    % Calcolo delle matrici A(i-1,i) per il giunto i-esimo. Queste vengono
    % collocate una dietro l'altra nella matrice tridimensionale T.
    % In generale T(:,:,i) sarà la matrice di trasformazione omogenea A(i-1,i)
    %
    %  0 --------- 0 --------- 0 --------- 0 
    % i=0         i=1         i=2         i=3
    %
    % Analizziamo i termin:
    %
    % T(:,:,1) = A(0,1) = [i=0 -> i=1]
    %  x --------- x --------- 0 --------- 0 
    % i=0         i=1         i=2         i=3
    %
    % T(:,:,2) = A(1,2) = [i=1 -> i=2]
    %  0 --------- x --------- x --------- 0 
    % i=0         i=1         i=2         i=3
    %
    % T(:,:,3) = A(2,3) = [i=2 -> i=3]
    %  0 --------- 0 --------- x --------- x 
    % i=0         i=1         i=2         i=3
    
    % slide 37 robotics02.pdf
    for i=1:n
        T(:,:,i) = Homogeneous(DH(i,:));
    end

    % Homogeneous transformation matrices with respect to the arm base 
    % frame. T0(:,:,n) contains the homogeneous transformation matrix 
    % between the end effector and the arm base frame
    
    % T0 conterrà i prodotti delle matrici di trasformazione omogenea dalla
    % terna 0 ad ogni giunto i-esimo
    % In generale T0(:,:,i) sarà la matrice di trasformazione omogenea
    % dalla terna 0 al giunto i-esimo
    %
    %  0 --------- 0 --------- 0 --------- 0 
    % i=0         i=1         i=2         i=3
    %
    % Analizziamo i termini:
    %
    % T0(:,:,1) = T(:,:,1)                       = A(0,1)                            = [i=0 -> i=1]
    %  x --------- x --------- 0 --------- 0 
    % i=0         i=1         i=2         i=3
    %
    % T0(:,:,2) = T(:,:,1) * T(:,:,2)            = A(0,2) = A(0,1) * A(1,2)          = [i=0 -> i=2]
    %  x --------- 0 --------- x --------- 0 
    % i=0         i=1         i=2         i=3
    %
    % T0(:,:,3) = T(:,:,1) * T(:,:,2) * T(:,:,3) = A(0,3) = A(0,1) * A(1,2) * A(2,3) = [i=0 -> i=3]
    %  x --------- 0 --------- 0 --------- x 
    % i=0         i=1         i=2         i=3
    %
    
    % La T0(:,:,1) è inizializzata alla T(:,:,1)
    % Il ciclo così partirà da i=2 e computerà tutte le trasformazioni
    % dalla terna 0 alla terna i-esima
    T0(:,:,1) = T(:,:,1);
    
    % slide 37 robotics02.pdf
    % T0(:,:,i) = T(:,:,0) * ... * T(:,:,i)              = A(0,1) * ... * A(i-1,i)
    %           = T(:,:,0) * ... * T(:,:,i-1) * T(:,:,i) = A(0,1) * ... * A(i-2,i-1) * A(i-1,i)
    %           = T0(:,:,i-1) * T(:,:,i)                 = A(0,i-1) * A(i-1,i)
    % matrice di trasformazione omogenea generica base -> giunto i-esimo
    for i=2:n
        T0(:,:,i) = T0(:,:,i-1) * T(:,:,i);
    end

end
