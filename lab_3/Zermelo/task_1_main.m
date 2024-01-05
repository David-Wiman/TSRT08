% Variables
tf = 1;
w = 1;

% Set ODE options
initial_conditions = [0, 0];
time_interval = [0, tf];

% Compute ODE
[time, state] = ode45(@(t, state) task_1_ODE(t, state, tf, w), time_interval, initial_conditions);

% Extract all states for plotting
x = state(:,1);
y = state(:,2);

% Plot
figure(1)
plot(x, y);
title('Optimal Trajectory')
xlabel('x')
ylabel('y')
grid