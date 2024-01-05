% Function to propagate the state
function next_state = task_1_ODE(t, state, tf, w)
    % Rename state components for readability
    x = state(1);
    y = state(2);
    
    % Compute theta
    theta = atan(tf - t);
    
    % State dynamics
    next_state(1) = w*cos(theta) + y;
    next_state(2) = w*sin(theta);
    
    % Transpose to get row vector 
    next_state = next_state';
end