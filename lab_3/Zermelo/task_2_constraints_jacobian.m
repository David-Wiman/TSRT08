function [a, constraints, c, constraints_jacobian] = ...
          task_2_constraints_jacobian(opt_vector, Ts)
    % Constants
    nr_states = 2;
    nr_var = 3;
    N = length(opt_vector)/nr_var - 1;
    
    a = [];
    c = [];
    
    % Allocate space
    constraints = zeros(nr_states*N, 1);
    constraints_jacobian = zeros(nr_states*N, nr_var*N);
    
    for time_instance = 0:N-1
        % Constraints
        state = opt_vector((1:2) + time_instance*nr_var);
        theta = opt_vector(3 + time_instance*nr_var);
        measured_next_state = opt_vector((1:2) + (time_instance+1)*nr_var);
        computed_next_state = task_2_ODE(state, theta, Ts);
        
        constraints((1:2) + time_instance*nr_states) = measured_next_state - computed_next_state;
        
        % Constraints_jacobian
        n = time_instance*nr_states;
        k = time_instance*nr_var;
        constraints_jacobian(1 + n, 1 + k) = -1;
        constraints_jacobian(1 + n, 2 + k) = -Ts;
        constraints_jacobian(1 + n, 3 + k) = Ts*sin(theta);
        constraints_jacobian(1 + n, 4 + k) = 1;
        constraints_jacobian(1 + n, 5 + k) = 0;
        constraints_jacobian(1 + n, 6 + k) = 0;

        constraints_jacobian(2 + n, 1 + k) = 0;
        constraints_jacobian(2 + n, 2 + k) = -1;
        constraints_jacobian(2 + n, 3 + k) = -Ts*cos(theta);
        constraints_jacobian(2 + n, 4 + k) = 0;
        constraints_jacobian(2 + n, 5 + k) = 1;
        constraints_jacobian(2 + n, 6 + k) = 0;
    end
    constraints = constraints';
    constraints_jacobian = constraints_jacobian';
end