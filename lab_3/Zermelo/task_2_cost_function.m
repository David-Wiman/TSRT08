function [Jacobian, gradient] = task_2_cost_function(opt_vector, Ts)
    % Cost function
    Jacobian = -opt_vector(end - 2); % F = -x(tf)
    gradient = zeros(size(opt_vector));
    gradient(end - 2) = -1;
end 