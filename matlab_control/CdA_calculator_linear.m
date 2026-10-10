% Proper inputs for this function are the sampled state vector, the list of
% precalculated CdA values, and matrices of position and velocity values 
% such that each row contains a single trajectory. These trajectories going
% from lowest to highest row indices should correspond to the lowest to
% highest element indices of the CdA values vector.

function CdA = CdA_calculator_linear(X, CdAs, X_matrix, V_matrix)

    % Assign position and velocity to the appropriate elements of the state
    % vector                                                                                                                                                        
    x = X(1);                                                                            
    v = X(2);                                                                            
    
    % preallocate vector of calculated velocities and velocity errors
    velocities = zeros(1, length(CdAs));
    velocity_errors = zeros(1, length(CdAs));
    
    % loop through each row to calculate the velocity using the data for
    % each CdA trajectory
    for idx = 1:size(X_matrix, 1)
    
        % find index of position point in the increasing Xs row directly below 
        % or equal to the sampled value
        index = length(X_matrix(idx, :)) - discretize(x, flip(X_matrix(idx, :)));
    
        % define first order interpolation values
        m = (V_matrix(idx, index + 1) - V_matrix(idx, index))/(X_matrix(idx, ...
            index + 1) - X_matrix(idx, index));
        b = V_matrix(idx, index);
        x_scaled = x - X_matrix(idx, index);
    
        % calculate velocity
        velocity = m*x_scaled + b;
    
        % fill velocity vector
        velocities(idx) = velocity;
    end
    
    % loop through all the calculated velocities and assemble a vector of
    % velocity errors 
    for idx = 1:length(velocities)
    
        velocity_errors(idx) = abs(velocities(idx) - v);
    end
    
    % return CdA value of the trajectory with the smallest error
    CdA = CdAs(velocity_errors == min(velocity_errors));
    
    % plot velocities of the trajectory associated with the smallest error
    % and the sampled velocity to see how close our calculation is
    hold on
    plot(X_matrix(CdAs == CdA, :), V_matrix(CdAs == CdA, :), '-')
    plot(x, v, 'o')
    xlabel('Position (m)')
    ylabel('Velocity (m/s)')
    title('Sampled Data and Precalculated Velocity Comparison')
    legend('Precalculated Velocity Trajectory', 'Sampled Data Point')
    hold off
    
    % print error
    fprintf('The error between the sampled and closest precalculated value is %.2e\n', min(velocity_errors))
end



