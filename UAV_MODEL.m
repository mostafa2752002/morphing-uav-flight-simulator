function out = UAV_MODEL(x, delta, wind, P)
    global P
    %% === Extract State Variables ===
    u     = x(1);   v     = x(2);   w     = x(3);
    phi   = x(4);   theta = x(5);   psi   = x(6);
    p     = x(7);   q     = x(8);   r     = x(9);

    %% === Extract Control Inputs ===
    delta_e = delta(1);    delta_a = delta(2);
    delta_r = delta(3);    delta_t = delta(4);

    %% === Extract Wind Inputs ===
    w_ns = wind(1);   % steady wind - North
    w_es = wind(2);   % steady wind - East
    w_ds = wind(3);   % steady wind - Down
    u_wg = wind(4);   % gust along body x-axis
    v_wg = wind(5);   % gust along body y-axis    
    w_wg = wind(6);   % gust along body z-axis

    %% === Rotation Matrix (NED to Body) ===
    sp = sin(phi);   cp = cos(phi);
    st = sin(theta); ct = cos(theta);
    ss = sin(psi);   cs = cos(psi);
    tt = tan(theta);

    rotation_to_body = [ ct*cs,           ct*ss,         -st;
                         sp*st*cs-cp*ss,  sp*st*ss+cp*cs, sp*ct;
                         cp*st*cs+sp*ss,  cp*st*ss-sp*cs, cp*ct ];

    %% === Wind in Body Frame ===
    V_wind_body = rotation_to_body * [w_ns; w_es; w_ds] + [u_wg; v_wg; w_wg];
    V_airspeed_body = [u; v; w] - V_wind_body;

    %% === Air Data Computation ===
    u_r = V_airspeed_body(1);
    v_r = V_airspeed_body(2);
    w_r = V_airspeed_body(3);

    Va    = sqrt(u_r^2 + v_r^2 + w_r^2);    % Airspeed
    alpha = atan(w_r / u_r);               % Angle of attack
    beta  = asin(v_r / Va);                % Sideslip angle

    %% === Lift Curve Blending Function (Sigma) ===
    sigma = (1 + exp(-P.M * (alpha - P.alpha0)) + exp(P.M * (alpha + P.alpha0))) ...
          / ((1 + exp(-P.M * (alpha - P.alpha0))) * (1 + exp(P.M * (alpha + P.alpha0))));

    %% === Aerodynamic Coefficients ===
    C_L = P.C_L_0 + P.C_L_alpha * alpha;

    C_D = P.C_D_p + ((P.C_L_0 + P.C_L_alpha * alpha)^2) / ...
          (pi * P.e * P.b^2 / P.S_wing);

    C_X           = -C_D * cos(alpha) + C_L * sin(alpha);
    C_X_q         = -P.C_D_q * cos(alpha) + P.C_L_q * sin(alpha);
    C_X_delta_e   = -P.C_D_delta_e * cos(alpha) + P.C_L_delta_e * sin(alpha);

    C_Z           = -C_D * sin(alpha) - C_L * cos(alpha);
    C_Z_q         = -P.C_D_q * sin(alpha) - P.C_L_q * cos(alpha);
    C_Z_delta_e   = -P.C_D_delta_e * sin(alpha) - P.C_L_delta_e * cos(alpha);

    %% === Forces in Body Frame ===
    Force(1) = -P.mass * P.gravity * sin(theta) ...
               + 0.5 * P.rho * Va^2 * P.S_wing * ...
               (C_X + C_X_q * P.c * q / (2 * Va) + C_X_delta_e * delta_e) ...
               + 0.5 * P.rho * P.S_prop * P.C_prop * ...
               ((P.k_motor * delta_t)^2 - Va^2);

    Force(2) =  P.mass * P.gravity * cos(theta) * sin(phi) ...
               + 0.5 * P.rho * Va^2 * P.S_wing * ...
               (P.C_Y_0 + P.C_Y_beta * beta + P.C_Y_p * P.b * p / (2 * Va) ...
               + P.C_Y_r * P.b * r / (2 * Va) + P.C_Y_delta_a * delta_a + P.C_Y_delta_r * delta_r);

    Force(3) =  P.mass * P.gravity * cos(theta) * cos(phi) ...
               + 0.5 * P.rho * Va^2 * P.S_wing * ...
               (C_Z + C_Z_q * P.c * q / (2 * Va) + C_Z_delta_e * delta_e);

    %% === Moments in Body Frame ===
    Torque(1) = 0.5 * P.rho * Va^2 * P.S_wing ...
                * (P.b * (P.C_ell_0 + P.C_ell_beta * beta + P.C_ell_p * P.b * p / (2 * Va) ...
                + P.C_ell_r * P.b * r / (2 * Va) + P.C_ell_delta_a * delta_a + P.C_ell_delta_r * delta_r)); ...
                - P.k_T_P * (P.k_Omega * delta_t)^2;

    Torque(2) = 0.5 * P.rho * Va^2 * P.S_wing * ...
               (P.c * (P.C_m_0 + P.C_m_alpha * alpha ...
               + P.C_m_q * P.c * q / (2 * Va) + P.C_m_delta_e * delta_e));

    Torque(3) = 0.5 * P.rho * Va^2 * P.S_wing * ...
               (P.b * (P.C_n_0 + P.C_n_beta * beta + P.C_n_p * P.b * p / (2 * Va) ...
               + P.C_n_r * P.b * r / (2 * Va) + P.C_n_delta_a * delta_a + P.C_n_delta_r * delta_r));

    %% === Translational Accelerations ===
    fx = Force(1);  fy = Force(2);  fz = Force(3);
    ell = Torque(1); m = Torque(2); n = Torque(3);

    udot = r * v - q * w + fx / P.mass;
    vdot = p * w - r * u + fy / P.mass;
    wdot = q * u - p * v + fz / P.mass;

    %% === Angular Rates ===
    rotation_angle = [1, sp*tt,     cp*tt;
                      0, cp,        -sp;
                      0, sp/ct,     cp/ct];

    angle_dot = rotation_angle * [p; q; r];

    phidot   = angle_dot(1);
    thetadot = angle_dot(2);
    psidot   = angle_dot(3);

    %% === Rotational Accelerations ===
    pdot = P.r1 * p * q - P.r2 * q * r + P.r3 * ell + P.r4 * n;
    qdot = P.r5 * p * r - P.r6 * (p^2 - r^2) + m / P.Jy;
    rdot = P.r7 * p * q - P.r1 * q * r + P.r4 * ell + P.r8 * n;

    %% === Output Vector ===
    xdot = [udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];

    out = [xdot;
           Force'; 
           Torque';
           Va;
           alpha;
           beta;
           0; 0; 0]; % wind NED (w_n, w_e, w_d), currently unused
end
