% Parameters
N = 50;
t_f = 1;
Ts = t_f/N;
nr_states = 2;
nr_var = 3;

% Constraints
init_state = [0 ; 0];

A = zeros(nr_states, nr_var*(N+1));
A(1:2, 1:2) = eye(2);
B = [0 ; 0];

% Initial guess
init_guess = [init_state ; 0]*ones(1, N+1);
init_guess = init_guess(:);

% Optimization options
options = optimset('fmincon');
options = optimset(options, 'Algorithm', 'interior-point');
options = optimset(options, 'GradObj', 'on');
options = optimset(options, 'GradConstr', 'on');
options = optimset(options, 'MaxFunEvals', 15000);

% Solve the problem
tic
[opt_vector, final_value, exit_flag, output] = fmincon(@task_2_cost_function, init_guess, [], [], A, B, ...
  [], [], @task_2_constraints_jacobian, options, Ts);
toc

% Show the result
fprintf('Final value of the objective function: %0.6f \n', final_value)

time = Ts*(0:N);
x = opt_vector(1:3:end)';
y = opt_vector(2:3:end)';
theta = opt_vector(3:3:end)';

figure(1)
subplot(2,1,1)
plot( time, x, 'b', time, y, 'g')
xlabel('Time')
legend('x','y', 'Location', 'northwest')
title('State Variables');
grid on

subplot(2,1,2)
plot( time(1:end-1), theta(1:end-1), 'b')
hold on
plot( time(1:end-1), atan(t_f - time(1:end-1)), '--r')
xlabel('Time')
ylabel('\theta')
legend('Computed control signal', 'Theoretical control signal')
title('Control Signal');
grid on