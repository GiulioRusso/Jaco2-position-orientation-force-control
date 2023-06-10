function tau_n=jaco2_n(q,dq)
%
% Computes the Coriolis+gravity terms for the KINOVA Jaco2
%
% function C = jaco2_n(q,dq)
%
% check the file jaco2Info_ver2.pdf for the definitions
%
% input:
%       q     dim 7x1     joint positions in DH convention
%       dq    dim 7x1     joint velocities in DH convention
% output:
%       n     dim 7x1     torques
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2022/2023

    tau_n = jaco2_C(q,dq)*dq + jaco2_g(q);

end
