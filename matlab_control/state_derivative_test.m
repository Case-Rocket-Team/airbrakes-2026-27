clear;
clc;
close all;

X0 = [2286; 0];
CdA = 0.016;
CdA_0 = 0.008;
CdA_1 = 0.03;

options = odeset('Events', @hit_ground_function);

[t, X, t_event, X_event] = ode45(@(t, X) state_derivative(t, X, CdA), [0, 40], X0, options);

figure(1)
plot(t, X)
title('State Over Time')
xlabel('Time(s)')
ylabel('Decomposed State Vector')
legend('Altitude (m)', 'Velocity (m/s)')

%%
figure(2)
Xs = [];
CdA_values = [];
Vs = [];
F_CdAs = {};
for idx = 1:100
    CdAs = linspace(CdA_0, CdA_1, 100);
    hold on
    [t, X, t_event, X_event] = ode45(@(t, X) state_derivative(t, X, CdAs(idx)), [0, 40], X0, options);

    Xs = [Xs; X];

    CdA_news = linspace(CdAs(idx), CdAs(idx), size(X, 1))';
    CdA_values = [CdA_values; CdA_news];

    plot(t, X)
    % Load velocity vector for each trajectory into successive rows
    Vs = [Vs; X(:, 2)']
    x_F_CdAs = flip(X(:, 1)');
    x_V_CdAs = flip(X(:, 2)');

    F_CdAs{idx} = griddedInterpolant(x_F_CdAs, x_V_CdAs, 'pchip', 'none');

    hold off
end
xlabel('Time')
ylabel('Decomposed State Vector')

length(Xs)
length(CdA_values)

F = scatteredInterpolant(Xs(:, 1), Xs(:, 2), CdA_values, 'linear', 'none');

figure(3)
plot(t, Vs)
length(Vs(:, 1))

length(F_CdAs)
length(CdAs)

save('CdA_function_data', 'F_CdAs', "CdAs")

Xs(50, :)

figure(4)

for idx = 1:length(F_CdAs)
    hold on
    plot(0:2286, F_CdAs{idx}(0:2286))
    hold off
end