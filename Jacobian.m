function J = Jacobian(DH)
%
% Computes the geometric Jacobian (only rotational joints)
%
% function J = Jacobian(DH)
%
% input:
%       DH     dim nx4     Denavit-Hartenberg table (a, alpha, d, theta)
%
% output:
%       J      dim 6xn     Geometric Jacobian
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

% p: 3xn matrix, the generic column is the position of frame i expressed in inertial frame
% z: 3xn matrix, the generic column is the z-versor of frame i expressed in inertial frame

n = size(DH,1); % numero di giunti
J = zeros(6,n);
% Jp = [jp1 ... jpn] 3xn contributo velocità lineare
% Jo = [jo1 ... jon] 3xn contributo velocità angolare
p = zeros(3,n);
z = zeros(3,n);

% compute homog. transf. from base frame
T0 = zeros(4,4,n);
for i=1:n
    
    % Calcolo le trasformazioni omogenee dalla base fino al giunto i-esimo
    % T0(:,:,i) che conterrà i prodotti delle matrici A(0,1) * ... *
    % A(i-1,i)
    
    % Calcolo la trasformazione omogenea A(i-1,i)
    T_i = Homogeneous(DH(i,:));
    if i==1
        % se i=1 la trasformazione omogenea da terna 0 a terna 1 è la sola
        % matrice di trasformazione omogenea T_i: A(0,1)
        T0(:,:,i) = T_i;
    else
        % altrimenti moltiplico la trasformazione A(i-1,i) a quelle
        % precedenti:
        % T0(:,:,i-1) = A(0,1) * ... * A(i-2,i-1)
        % T_i = A(i-1,i)
        T0(:,:,i) = T0(:,:,i-1)*T_i;
    end
    
    % slide 35 robotics02.pdf
    % Il vettore posizione della terna i-esima sarà pare ai primi 3 termini
    % della quarta colonna di T0(:,:,i)
    %
    % T0(0,i) = R(0,i) p(0,i)
    %           0 0 0     1
    %
    % R(0,i) dim 3x3
    % p(0,i) dim 3x1
    %
    p(:,i) = T0(1:3,4,i);
    
    % slide 31 robotics02.pdf
    % zi è la terza colonna del prodotto delle matrici di rotazione
    % R(0,i) = R(0,1) * ... * R(i-1,i) il prodotto delle matrici di
    % rotazione dalla terna 0 fino al giunto i-esimo
    %
    % R(0,i) = [xi, yi zi]
    %
    % xi, yi, zi dim 3x1
    %
    z(:,i) = T0(1:3,3,i);
end

z0 = [0 0 1]'; % l'asse z della terna base ha versore 1 in direzione z
p0 = [0 0 0]'; % il vettore posizione della terna base vale 0 (è l'origine)
% p(:,n) = pe (posizione organo terminale)
J(:,1) = [cross(z0,p(:,n)-p0);
            z0];
        
% slide 21 robotics03.pdf
% Jacobiano per giunto rotazionale
for i=2:n
    J(:,i) = [cross(z(:,i-1),p(:,n)-p(:,i-1));
            z(:,i-1)];
end





