function R = Rot_axisangle(r,theta)
%
% Rotation matrix assigned axis-angle
%
% function R = Rot_axisangle(r,theta)
%
% input:
%       r		dim 3x1     axis of rotation
%       theta   dim 1x1     angle of rotatRotion
%
% output:
%       R       dim 3x3     Rotation matrix
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

% slide 23 robotics02.pdf

r = r/norm(r);

ct = cos(theta);
st = sin(theta);
uct = 1-ct;
rx = r(1);
ry = r(2);
rz = r(3);

R(1,1) = rx^2*uct+ct;
R(1,2) = rx*ry*uct-rz*st;
R(1,3) = rx*rz*uct+ry*st;
R(2,1) = rx*ry*uct+rz*st;
R(2,2) = ry^2*uct+ct;
R(2,3) = ry*rz*uct-rx*st;
R(3,1) = rx*rz*uct-ry*st;
R(3,2) = ry*rz*uct+rx*st;
R(3,3) = rz^2*uct+ct;