function rpy = Rot2rpy(R)
%
% From rotation matrix to rpy (roll around z-axis as in the Siciliano's textbook)
%
% function rpy = Rot2Quat(R)
%
% input:
%       R		dim 3x3     rotation matrix
%
% output:
%       rpy     dim 3x1     rpy (roll around z-axis as in the Siciliano's textbook)
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

phi   = atan2(R(2,1),R(1,1));
theta = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
psi   = atan2(R(3,2),R(3,3));
    
rpy = [phi theta psi]';
