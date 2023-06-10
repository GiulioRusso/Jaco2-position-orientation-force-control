function eo = QuatError(ed,e)
%
% error quaternion
%
% eo = QuatError(ed,e)
%
% input:
%       ed     dim 4x1     desired quaternion (scalar in the end)
%       e      dim 4x1     actual quaternion (scalar in the end)
%
% output:
%       eo     dim 3x1     error quaternion 
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

% slide 102 robotics03.pdf
eo = e(4)*ed(1:3) - ed(4)*e(1:3) - cross(ed(1:3),e(1:3));

% Nel caso abbia rotazioni superiori a 2pi
eta_tilde = e(4)*ed(4) + e(1:3)'*ed(1:3);
if eta_tilde < 0
    eo = - eo;
end
