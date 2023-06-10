function T = Homogeneous(DH_row)
%
% Homogeneous transformation matrix between consecutive frames according to DH convention
%
% T = Homogeneous_dh(DH_i)
%
% input:
%       DH_row     dim 1x4     row i of the Denavit-Hartenberg table (a, alpha, d, theta)
%
% output:
%       T      dim 4x4      Homogeneous transformation matrix
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici,  2022/2023

    % Estraggo i valori (scalari) di a, alpha, d, theta del giunto i-esimo
    % dalla tabella DH
    a = DH_row(1);
    alpha = DH_row(2);
    d = DH_row(3);
    theta = DH_row(4);
    
    % Calcolo i seni e coseni necessari per la generica trasformazione
    % omogenea
    ct = cos(theta);
    st = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);
    
    % slide 43 robotics02.pdf
    % T(:,:,i) della DirectKinematics.m = A(i-1,i) = A(i-1,i')A(i',i)
    % Computo la generica trasformazione omogenea da i-1 a i
    T = [ct -st*ca st*sa a*ct;
         st ct*ca -ct*sa a*st;
         0   sa  ca  d;
         0   0   0   1];

end
