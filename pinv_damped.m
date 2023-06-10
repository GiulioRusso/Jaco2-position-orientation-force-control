function out = pinv_damped(J,W)
%
% Pseudoinversion, eventually damped
%
% function out = pinv_damped(J,W)
%
% input:
%       J      dim mxn     Jacobian
%       W      dim nxn     Weight matrix
%
% output:
%       out    dim nxm     (damped) pseudoinverse
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, (a, alpha, d, theta)

    l = 100*1e-2; % daming treshold selezionato dal professore
    [m,~]=size(J);
    if cond(J)<1e3 % condition treshold selezionato dal professore
        % Se il numero di condizione è "piccolo" per come lo abbiamo
        % inteso, allora ho una semplice pseudoinversa pesata
        out = W*J'/(J*W*J');
    else
        % altrimenti ho una pseudoinversa smorzata con parametro l
        out = W*J'/(l*eye(m) + J*W*J');
    end

end


