function [ts, steps, Xs, CdAs] = controller_sampled(X0, F, t0, tf, number_updates)

    steps = linspace(t0, tf, number_updates);
    CdA0 = F(X0(1), -X0(2));
    Xs = [0, 0];
    ts = [0];
    CdAs = [CdA0];

    for idx = 1:length(steps)-1

        [t, X] = ode45(@(t, X) state_derivative(t, X, CdA0), [steps(idx), steps(idx + 1)], X0);
        
        X0 = X(end, :)';

        CdA0 = F(X0(1), -X0(2));
        CdAs = [CdAs, CdA0];

        Xs(end, :) = [];
        Xs = [Xs; X];
        ts(end) = [];
        ts = [ts; t];
    end
end

        
