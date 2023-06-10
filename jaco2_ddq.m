function ddq = jaco2_ddq(tau,q,dq)
%
% Computes the direct dynamics for the Kinova JAco2
%
% function ddq = jaco2_ddq(tau,q,dq)
%
% check the file jaco2Info_ver2.pdf for the definitions
%
% input (in DH convention):
%       tau     dim 7x1     torques
%       q       dim 7x1     joint positions
%       dq      dim 7x1     joint velocities
% output:
%       ddq     dim 7x1     joint accelerations
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023


    n = jaco2_n(q,dq);
    B = jaco2_B(q);
    ddq = B\(tau - n);

end

