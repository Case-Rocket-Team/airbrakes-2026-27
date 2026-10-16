clear;
clc;
close all;

X0 = [503.73; 250.856];
X0_sim = [2286; 0];
CdA_0 = 0;
CdA_f = 0.01;
number_CdAs = 500;

F = CdA_interpolator(X0_sim, CdA_0, CdA_f, number_CdAs);

t0 = 0;
tf = 60;
number_updates = 2000;

[ts, steps, Xs, CdAs] = controller_sampled(X0, F, t0, tf, number_updates);

figure(1)
hold on
plot(ts, Xs(:, 1), '-b')
plot(ts, linspace(2286, 2286, length(ts)), '--r')
xlabel('Time (s)')
ylabel('Altitude (m)')
title('Sampled Data Trajectory')
legend('Flight Trajectory', 'Altitude Target')
hold off

figure(2)
plot(steps, CdAs, 'ob')
xlabel('Time (s)')
ylabel('CdA Values')
title('CdA Values Over Time')
