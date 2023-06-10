function tau = jaco2_tau(q,dq,ddq)
%
% Computes the inverse dynamics for the Kinova Jaco2
%
% function tau = jaco2_tau(q,dq,ddq)
%
% check the file jaco2Info_ver2.pdf for the definitions
%
% input (in DH convention):
%       q       dim 7x1     joint positions
%       dq      dim 7x1     joint velocities
%       ddq     dim 7x1     joint accelerations
% output:
%       tau     dim 7x1     torques
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

    n = jaco2_n(q,dq);
    B = jaco2_B(q);
    tau = B*ddq + n;

end

