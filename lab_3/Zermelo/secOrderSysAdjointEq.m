function dlambda = secOrderSysAdjointEq(t, lambda, tu, u, tx, x)
% secOrderSysAdjointEq - Continuous-time adjoint equation
%
% t         - Time
% lambda    - Adjoint variable at time t
% tu        - Discrete-time time vector for input vector
% u         - Discrete-time input vecor
% tx        - Discrete-time time vector for state vector
% x         - Discrete-time state vecor
% dx - Time derivative of state at time t

% Compute the input and state at time t by interpolating the discrete-time
% input and state vectors
u = interp1(tu,u,t); 
x = interp1(tx,x,t); 

dlambda = [0 ; -lambda(1)];
