function CdA = CdA_calculator(X, F_CdAs, CdAs)
    
    x = X(1);
    disp(x)
    v = X(2);
    
    velocity_errors = [];

    for idx = 1:length(CdAs)

        velocity_error = abs(F_CdAs{idx}(x) - v);
        velocity_errors = [velocity_errors, velocity_error];

    end
    
    CdA = CdAs(velocity_errors == min(velocity_errors));

    figure;
    hold on
    plot(0:2286, F_CdAs{velocity_errors == min(velocity_errors)}(0:2286));
    plot(x, v,'o')
    grid on;
    xlabel('Altitude (m)')
    ylabel('Velocity (m/s)')
    title('Sampled Compared to Calculated Values')
    legend('Nearest Precalculated Velocity Trajectory', 'Sampled Data Point')

    fprintf('The minimum velocity error between the sampled state and the calculated value is %.2f', min(velocity_errors))
end
    
