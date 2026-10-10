%D=Cd⋅ρ⋅V2⋅A2

function Xdot = state_derivative(t, X, CdA)

    x = X(1);
    v = X(2);
    
    g = 9.81; % m/s

    Mr = 34; % kg

    p = 1.225*exp(-x/8500);

    F_drag = -(CdA*p*v^2)/2;

    F_gravity = -Mr*g;
    

    a = (F_drag + F_gravity)/Mr;

    Xdot = [v; a];

end

