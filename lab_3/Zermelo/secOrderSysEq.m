function dx = secOrderSysEq(t, x, tu, u)
% secOrderSysEq - Continuous-time dynamic equation
%
% t  - Time
% x  - State at time t
% tu - Discrete-time time vector
% u  - Discrete-time input vecor
% dx - Time derivative of state at time t

% Compute the input at time t by interpolating the discrete-time inout
% vector
u = interp1(tu, u, t);

dx = [cos(u) + x(2) ; sin(u)];
