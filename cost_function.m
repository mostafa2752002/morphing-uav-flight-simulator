function [F0] = cost_function(Z)
    % cost_function: Computes the cost for trim optimization
    % INPUT: Z  - decision variable vector [states(1:9); inputs(10:13)]
    % OUTPUT:    %   F0 - scalar cost value
    global P

    X = Z(1:9);       % State vector [pn; pe; pd; u; v; w; phi; theta; psi]
    U = Z(10:13);     % Control inputs [delta_e; delta_a; delta_r; delta_t]

    % Call UAV model to get state derivatives and outputs
    % Assuming no wind disturbance [0 0 0 0 0 0]
    out = UAV_MODEL(X, U, [0 0 0 0 0 0], P);
    xdot = out(1:9);      % Time derivative of the state vector
    Va    = out(16);      % Airspeed
    alpha = out(17);      % Angle of attack
    
    % Compute flight path angle gamma = theta - alpha
    theta = X(5);         
    gamma = theta - alpha;
    % Build error vector Q for trim conditions
    
    Q = [ xdot;        % Want steady flight: xdot ≈ 0
          Va - 11.2;     % Desired airspeed = 15 m/s
          gamma;       % Desired gamma = 0 (level flight)
          X(2);        % Y-position ≈ 0 (pe)
          X(4);        % Lateral velocity v ≈ 0
          X(6) ];      % Roll rate p ≈ 0
    
    % Define weighting matrix H (identity matrix here)
    H = diag(ones(size(Q')));
    % Compute quadratic cost function
    F0 = Q' * H * Q;
end
