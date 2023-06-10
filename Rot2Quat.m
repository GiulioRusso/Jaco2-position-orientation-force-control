function e = Rot2Quat(R)
%
% From rotation matrix to quaternions
%
% function e = Rot2Quat(R)
%
% input:
%       R		dim 3x3     rotation matrix
%
% output:
%       e       dim 4x1      quaternion (scalar in the end)
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

% slide 27 robotics02.pdf

e = zeros(4,1);

% scalare
e(4) = .5*sqrt(R(1,1)+R(2,2)+R(3,3)+1);

if (R(3,2)-R(2,3))>=0
    e(1) = .5*sqrt(R(1,1)-R(2,2)-R(3,3)+1);
else
    e(1) = -.5*sqrt(R(1,1)-R(2,2)-R(3,3)+1);
end

if (R(1,3)-R(3,1))>=0
    e(2) = .5*sqrt(-R(1,1)+R(2,2)-R(3,3)+1);
else
    e(2) = -.5*sqrt(-R(1,1)+R(2,2)-R(3,3)+1);
end

if (R(2,1)-R(1,2))>=0
    e(3) = .5*sqrt(-R(1,1)-R(2,2)+R(3,3)+1);
else
    e(3) = -.5*sqrt(-R(1,1)-R(2,2)+R(3,3)+1);
end

% Matlab può assegnare parte immaginaria di 10^-16, per cui la levo a
% priori. Può essere un problema se dopo tante integrazioni quel 10^-16
% diventa un 10^-3
e=real(e);
% Normalizza (non cambia direzione, ma sono sicuro che ho norma unitaria)
e = e/norm(e);

