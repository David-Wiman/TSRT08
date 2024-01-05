function Hu = secOrderSysGradient(tLa, La, tu, u, tx, x)
% secOrderSysGradient - Computes the gradient of the Hamiltonian with
% respect to u
%
% tLa - Discrete-time time vector for input vector
% La  - Discrete-time input vecor
% tu  - Discrete-time time vector for input vector
% u   - Discrete-time input vecor
% tx  - Discrete-time time vector for state vector
% x   - Discrete-time state vecor
% Hu  - Derivative of Hamiltonian with respect to u

% Compute the input and state at time t by interpolating the discrete-time
% input and state vectors

% States at time tu
x1 = interp1(tx, x(:,1), tu); 
x2 = interp1(tx, x(:,2), tu);
% Adjoint variables at time tu
la1 = interp1(tLa, La(:,1), tu);
la2 = interp1(tLa, La(:,2), tu);

Hu = -la1.*sin(u) + la2.*cos(u);