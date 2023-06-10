function TA = ComputeTA(euler,or_type)
%
% Compute TA for the convertion from geometric to analitycal Jacobian
%
% function TA = computeTA(rpy)
%
% input:
%       euler   dim 3x1     rpy or zyz (roll around z-axis as in the Siciliano's textbook)
%       or_type string      'rpy', 'zyz'
%
% output:
%       TA      dim 6x6     [v; omega] = TA*[v; euler]
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

cf = cos(euler(1)); sf = sin(euler(1));
ct = cos(euler(2)); st = sin(euler(2));

if strcmp(or_type,'rpy')
    TA = [0 -sf cf*ct 
          0  cf sf*ct
          1  0  -st];
elseif strcmp(or_type,'zyz')
    TA = [0 -sf cf*st 
          0  cf sf*st
          1  0  ct];
end
TA = blkdiag(eye(3,3),TA);

