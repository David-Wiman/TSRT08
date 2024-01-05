% Second order optimal control problem solved with a gradient method.
% The final state constraint is added to the cost function as a penalty. 
N = 2000; % number of sample points
tf = 1; % final time
h = tf/N; % sample time
c = 200; % cost on the terminal constraint
xi = [0,0]'; % initial constraint
%%% Optimization options
maxIter = 200; % maximum number of iterations
alpha = 0.01; % Step size in the gradient update step, increased from 0.001
tols = 1e-3;
%%% Start guess
tu = 0:h:tf;
u = linspace(0, 1, length(tu));
%%% Solve the problem
iter = 1;
dua = 1;

tic
while dua > tols
  %%% Simulate system forward
  [tx,x] = ode23(@secOrderSysEq, [0, tf], xi, [], tu, u);
  %%% Compute final constraint for lambda
  Laf = secOrderSysFinalLambda(x, [0 ; 0], c);
  %%% Simulate adjoint system backwards
  [tLa,La] = ode23(@secOrderSysAdjointEq, [tf,0], Laf, [], tx, x, tu, u);
  %%% Compute gradient and update the control signal
  Hu = secOrderSysGradient(tLa, La, tu, u, tx, x);
  du = alpha*Hu;
  u = atan(tf - tu); % from 1.1
  %%% Abort the optimization
  if iter > maxIter
      break;
  end
  iter = iter + 1;
  dua = norm(du)/sqrt(length(du)); 
end
toc

%%% Show the result
y = x(:,2)';
x = x(:,1)';
fval = -x(end);

fprintf('Final value of the objective function: %0.6f \n', fval)

figure(1)
subplot(2,1,1)
plot( tx, x, 'b')
hold on
plot( tx, y, 'g')
xlabel('Time')
title('State Variables');
legend('x','y', 'Location', 'northwest')
grid on

subplot(2,1,2)
plot( tu, u, 'b')
hold on
plot (tu, atan(tf - tu), 'r--')
xlabel('Time')
ylabel('\theta')
title('Control Signal');
legend('Computed control signal', 'Theoretical control signal')
grid on
