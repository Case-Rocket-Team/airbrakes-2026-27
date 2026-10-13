clear;
clc;
close all;

X0 = [2286; 0];
CdA = 0.016;
CdA_0 = 0.008;
CdA_1 = 0.03;

options = odeset('Events', @hit_ground_function);

[t, X, t_event, X_event] = ode45(@(t, X) state_derivative(t, X, CdA), [0, 40], X0, options);

x_v_cda = [];

for idx = 1:100
    CdAs = linspace(CdA_0, CdA_1, 100);

    [t, X, t_event, X_event] = ode45(@(t, X) state_derivative(t, X, CdAs(idx)), [0, 40], X0, options);
    
    % make matrix of x and v values in rows 1 and 2 with the CdA value for
    % the pair in row 3
    col_three_cdas = [];
    for jdx = 1:size(X, 1)
        col_three_cdas = [col_three_cdas, CdAs(idx)];
    end
    
    x_v_cda = [x_v_cda; [X, col_three_cdas']];
end

[unique_Xs_values, ia] = unique(x_v_cda(:, 1:2), 'rows', 'stable');

duplicate_idx = [];

for idx = 1:size(x_v_cda(:, 1:2), 1)
    if ismember(idx, ia) == false
        duplicate_idx = [duplicate_idx, idx];
    end
end

x_v_cda(duplicate_idx, :) = [];
F = scatteredInterpolant(x_v_cda(:, 1), x_v_cda(:, 2), x_v_cda(:, 3), 'natural', 'none');

% make plot specifications
x_range = linspace(min(x_v_cda(:,1)), max(x_v_cda(:,1)), 200);
v_range = linspace(min(x_v_cda(:,2)), max(x_v_cda(:,2)), 200);

[X_grid, V_grid] = meshgrid(x_range, v_range);

CdA_grid = F(X_grid, V_grid);

figure
surf(X_grid, V_grid, CdA_grid, 'EdgeColor', 'none')

xlabel('Position (m)')
ylabel('Velocity (m/s)')
zlabel('CdA (m^2)')
title('Interpolated CdA Surface')

colorbar
view(3)