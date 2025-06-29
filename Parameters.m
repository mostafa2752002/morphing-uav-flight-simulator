clear; close all;
global P
%% === Mass and Inertia ===
P.gravity = 9.8;                  % gravitational acceleration [m/s^2]
P.mass = 3.066;                   % mass of the UAV [kg]
P.Jx = 0.40799329024;             % moment of inertia about x-axis [kg·m^2]
P.Jy = 0.13120186789;             % moment of inertia about y-axis [kg·m^2]
P.Jz = 0.50848672281;             % moment of inertia about z-axis [kg·m^2]
P.Jxz = 0.02958930522;            % product of inertia [kg·m^2]

%% === Aerodynamic Geometry and Constants ===
P.b = 1.5339;                     % wingspan [m]
P.c = 0.2773;                     % mean aerodynamic chord [m]
P.S_wing        = P.b * P.c;%
P.S_prop        = 0.0613;       % = 0.2027;
P.rho           = 1.2682;
P.k_motor = 80;                   % motor constant
P.k_T_P = 0;                      % torque-thrust constant (unused here)
P.k_Omega = 0;                    % motor angular velocity constant (unused)
P.e = 0.85;                       % Oswald efficiency factor

%% === Longitudinal Aerodynamic Coefficients ===
P.C_L_0 = 0.201;
P.C_L_alpha = 4.769099;
P.C_L_q = 7.366996;
P.C_L_delta_e = 0.009689 * 180/pi;

P.C_D_0 = 0.02399;
P.C_D_alpha = 0.30;
P.C_D_p = 0.0437;
P.C_D_q = 0.0;
P.C_D_delta_e = 0.0;

P.C_m_0 = 0.044;
P.C_m_alpha = -0.61317;
P.C_m_q = -7.14822;
P.C_m_delta_e = -0.019221 * 180/pi;

%% === Lateral-Directional Aerodynamic Coefficients ===
P.C_Y_0 = 0.0;
P.C_Y_beta = -0.219066;
P.C_Y_p = 0.106083;
P.C_Y_r = 0.144797;
P.C_Y_delta_a = -0.000218 * 180/pi;
P.C_Y_delta_r = 0.002507 * 180/pi;

P.C_ell_0 = 0.0;
P.C_ell_beta = -0.06544;
P.C_ell_p = -0.44893;
P.C_ell_r = 0.144185;
P.C_ell_delta_a = 0.005176 * 180/pi;
P.C_ell_delta_r = 0.000237 * 180/pi;

P.C_n_0 = 0.0;
P.C_n_beta = 0.070835;
P.C_n_p = -0.033581;
P.C_n_r = -0.052576;
P.C_n_delta_a = -0.000066 * 180/pi;
P.C_n_delta_r = -0.000829 * 180/pi;

%% === Propulsion Parameters ===
P.C_prop = 1.0;
P.M = 50;                          % sharpness factor in C_L(alpha) transition
P.epsilon = 0.1592;                % offset in angle of attack [rad]
P.alpha0 = -0.0346;                % zero-lift angle of attack [rad]

%% === Initial Conditions ===
P.pn0 = 0;                         % initial North position [m]
P.pe0 = 0;                         % initial East position [m]
P.pd0 = -100;                      % initial Down position [m] (negative = altitude)

P.u0 = 15;                         % initial body x-axis velocity [m/s]
P.v0 = 0;                          % initial body y-axis velocity [m/s]
P.w0 = 0;                          % initial body z-axis velocity [m/s]

P.phi0 = 0;                        % initial roll angle [rad]
P.theta0 = 0;                      % initial pitch angle [rad]
P.psi0 = 0;                        % initial yaw angle [rad]

P.p0 = 0;                          % initial roll rate [rad/s]
P.q0 = 0;                          % initial pitch rate [rad/s]
P.r0 = 0;                          % initial yaw rate [rad/s]

%% === Gamma Parameters (from uavbook, p. 36) ===
P.Gamma  = P.Jx*P.Jz - P.Jxz^2;
P.Gamma1 = (P.Jxz*(P.Jx - P.Jy + P.Jz)) / P.Gamma;
P.Gamma2 = (P.Jz*(P.Jz - P.Jy) + P.Jxz^2) / P.Gamma;
P.Gamma3 = P.Jz / P.Gamma;
P.Gamma4 = P.Jxz / P.Gamma;
P.Gamma5 = (P.Jz - P.Jx) / P.Jy;
P.Gamma6 = P.Jxz / P.Jy;
P.Gamma7 = (P.Jx*(P.Jx - P.Jy) + P.Jxz^2) / P.Gamma;
P.Gamma8 = P.Jx / P.Gamma;

%% === Wind and Turbulence Parameters ===
P.wind_n = 0;                      % steady wind - North component [m/s]
P.wind_e = 0;                      % steady wind - East component [m/s]
P.wind_d = 0;                      % steady wind - Down component [m/s]

w_ns = 0;                        % steady wind - North
w_es = 0;                        % steady wind - East
w_ds = 0;                        % steady wind - Down
u_wg = 0;                        % gust along body x-axis
v_wg = 0;                        % gust along body y-axis    
w_wg = 0;                        % gust along body z-axis

P.L_u = 200;                       % turbulence scale length in u-direction [m]
P.L_v = 200;                       % turbulence scale length in v-direction [m]
P.L_w = 50;                        % turbulence scale length in w-direction [m]

P.sigma_u = 1.06;                  % turbulence intensity in u-direction [m/s]
P.sigma_v = 1.06;                  % turbulence intensity in v-direction [m/s]
P.sigma_w = 0.7;                   % turbulence intensity in w-direction [m/s]

%% === r1–r8 Inertia Coupling Terms (for convenience) ===
P.r  = P.Jx * P.Jz - P.Jxz^2;
P.r1 = P.Jxz * (P.Jx - P.Jy + P.Jz) / P.r;
P.r2 = P.Jz * (P.Jz - P.Jy) + P.Jxz^2;
P.r3 = P.Jz / P.r;
P.r4 = P.Jxz / P.r;
P.r5 = (P.Jz - P.Jx) / P.Jy;
P.r6 = P.Jxz / P.Jy;
P.r7 = ((P.Jx - P.Jy) * P.Jx + P.Jxz^2) / P.r;
P.r8 = P.Jx / P.r;

%% === Stability Derivatives: Roll (P) and Yaw (r) Moments ===

% Roll moment coefficients
P.C_P_0        = P.r3 * P.C_ell_0        + P.r4 * P.C_n_0;
P.C_P_beta     = P.r3 * P.C_ell_beta     + P.r4 * P.C_n_beta;
P.C_P_p        = P.r3 * P.C_ell_p        + P.r4 * P.C_n_p;
P.C_P_r        = P.r3 * P.C_ell_r        + P.r4 * P.C_n_r;
P.C_P_delta_a  = P.r3 * P.C_ell_delta_a  + P.r4 * P.C_n_delta_a;
P.C_P_delta_r  = P.r3 * P.C_ell_delta_r  + P.r4 * P.C_n_delta_r;

% Yaw moment coefficients
P.C_r_0        = P.r4 * P.C_ell_0        + P.r8 * P.C_n_0;
P.C_r_beta     = P.r4 * P.C_ell_beta     + P.r8 * P.C_n_beta;
P.C_r_p        = P.r4 * P.C_ell_p        + P.r8 * P.C_n_p;
P.C_r_r        = P.r4 * P.C_ell_r        + P.r8 * P.C_n_r;
P.C_r_delta_a  = P.r4 * P.C_ell_delta_a  + P.r8 * P.C_n_delta_a;
P.C_r_delta_r  = P.r4 * P.C_ell_delta_r  + P.r8 * P.C_n_delta_r;

delta_a = 0;
delta_e = 0;
delta_r = 0;
delta_th = 0;
