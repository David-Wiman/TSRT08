function next_state = task_2_ODE(state, theta, Ts)
    % Discrete system propagation, from task 1.2 (a)
    next_state = [1, Ts ; 0, 1]*state + [Ts*cos(theta) ; Ts*sin(theta)];
end