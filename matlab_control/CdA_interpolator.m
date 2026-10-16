function F = CdA_interpolator(X0, CdA_0, CdA_f, number_CdAs)

    options = odeset('Events', @hit_ground_function);
    
    x_v_cda = [];
    
    for idx = 1:number_CdAs
        CdAs = linspace(CdA_0, CdA_f, number_CdAs);
    
        [~, X, ~, ~] = ode45(@(t, X) state_derivative(t, X, CdAs(idx)), [0, 40], X0, options);
        
        % make matrix of x and v values in rows 1 and 2 with the CdA value for
        % the pair in row 3
        col_three_cdas = [];
        for jdx = 1:size(X, 1)
            col_three_cdas = [col_three_cdas, CdAs(idx)];
        end
        
        x_v_cda = [x_v_cda; [X, col_three_cdas']];
    end
    
    [~, ia] = unique(x_v_cda(:, 1:2), 'rows', 'stable');
    
    duplicate_idx = [];
    
    for idx = 1:size(x_v_cda(:, 1:2), 1)
        if ismember(idx, ia) == false
            duplicate_idx = [duplicate_idx, idx];
        end
    end
    
    x_v_cda(duplicate_idx, :) = [];
    F = scatteredInterpolant(x_v_cda(:, 1), x_v_cda(:, 2), x_v_cda(:, 3), 'natural', 'none');

end