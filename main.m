clear;clc;close all;
%% Run necessarys codes
run('Parameters.m');
%% Load Trim Data or Compute It
if isfile('trim_results.mat')
    load('trim_results.mat');  % Load x_trim and u_trim
else
    run('Trim_UAV.m');  % Run only if not previously saved
end
 x_trim=ZStar(1:9);
 u_trim=ZStar(10:13);

%run('run_linearlization.m')
%run('State_Space.m')
sim('Model_.slx');
export_fig=1;
export_fig2=1;
%% Getting data from Simulink
t = ans.tout;
%States
X1 = ans.simStates.Data(:,1);%u
X2 = ans.simStates.Data(:,2);%v
X3 = ans.simStates.Data(:,3);%w
X4 = ans.simStates.Data(:,4);%phi
X5 = ans.simStates.Data(:,5);%theta
X6 = ans.simStates.Data(:,6);%psi
X7 = ans.simStates.Data(:,7);%p
X8 = ans.simStates.Data(:,8);%q
X9 = ans.simStates.Data(:,9);%r

% Inputs
delta_e = ans.simInput.Data(:,1);
delta_a = ans.simInput.Data(:,2);
delta_r = ans.simInput.Data(:,3);
delta_t = ans.simInput.Data(:,4);

%Forces and Moments
Fx = ans.simForcesMoments.Data(:,1);
Fy = ans.simForcesMoments.Data(:,2);
Fz = ans.simForcesMoments.Data(:,3);
l  = ans.simForcesMoments.Data(:,4);
M  = ans.simForcesMoments.Data(:,5);
N  = ans.simForcesMoments.Data(:,6);

%Air Data
Va    = ans.simVaalphaBeta.Data(:,1);
alpha = ans.simVaalphaBeta.Data(:,2);
beta  = ans.simVaalphaBeta.Data(:,3);

%Inital frame data
Vn = ans.simVelocityPosition.Data(:,1); % velocity in north
Ve = ans.simVelocityPosition.Data(:,2); % velocity in east
Vd = ans.simVelocityPosition.Data(:,3); % velocity in down
Pn = ans.simVelocityPosition.Data(:,4); % position in north
Pe = ans.simVelocityPosition.Data(:,5); % position in east
Pd = ans.simVelocityPosition.Data(:,6); % position in down

%% 3D Flight Trajectory (Adjusted and Auto-Saved)
fig = figure('Name','3D Flight Trajectory (Adjusted)');

% Plot the 3D trajectory
plot3(Pn, Pe, -Pd, 'LineWidth', 1.5, 'Color', 'r')
set(gca, 'ZDir','reverse')  % Down is positive in flight dynamics
xlabel('North (m)'); ylabel('East (m)'); zlabel('Altitude (m)')
grid on; axis equal;

% Safe buffer calculations
x_range = max(Pn) - min(Pn);
y_range = max(Pe) - min(Pe);
z_range = max(-Pd) - min(-Pd);

x_buffer = max(1, 0.1 * x_range);
y_buffer = max(1, 0.1 * y_range);
z_buffer = max(1, 0.1 * z_range);

% Axis limits
xlim([min(Pn) - x_buffer, max(Pn) + x_buffer]);
ylim([min(Pe) - y_buffer, max(Pe) + y_buffer]);
zlim([min(-Pd) - z_buffer, max(-Pd) + z_buffer]);

% View and labels
view(3)
legend('Flight Path', 'Location', 'best')

% Auto-save the figure
exportgraphics(fig, '3D_Flight_Trajectory.png', 'BackgroundColor', 'none', 'ContentType', 'image');

%% Position in Initial Frame
fig = figure('Name','Position in Initial Frame');
% ---- North ----
subplot(3,1,1)
plot(t, Pn, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('Pos in North (m)')
grid on
grid minor
range_pn = max(Pn) - min(Pn);
buffer_pn = max(1, 0.05 * range_pn);
ylim([min(Pn) - buffer_pn, max(Pn) + buffer_pn])
% ---- East ----
subplot(3,1,2)
plot(t, Pe, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('Pos in East (m)')
grid on
grid minor
range_pe = max(Pe) - min(Pe);
buffer_pe = max(1, 0.05 * range_pe);
ylim([min(Pe) - buffer_pe, max(Pe) + buffer_pe])
% ---- Down ----
subplot(3,1,3)
plot(t, -Pd, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('Pos in Down (m)')
grid on
grid minor
range_pd = max(-Pd) - min(-Pd);
buffer_pd = max(1, 0.05 * range_pd);
ylim([min(-Pd) - buffer_pd, max(-Pd) + buffer_pd])

% ---- Auto-save ----
if export_fig == 1
    exportgraphics(fig, 'Position in Initial Frame.png', 'BackgroundColor', 'none', 'ContentType', 'image')
end
%% Velocity in Initial Frame
fig = figure('Name','Velocity in initial frame');

% --- North velocity ---
subplot(3,1,1)
plot(t, Vn, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('Velocity in North ($\frac{m}{s}$)', 'interpreter', 'latex')
grid on
grid minor
range_vn = max(Vn) - min(Vn);
buffer_vn = max(0.5, 0.05 * range_vn);
vmin = max(0, min(Vn));
vmax = max(Vn);
ylim([vmin - buffer_vn, vmax + buffer_vn])

% --- East velocity ---
subplot(3,1,2)
plot(t, Ve, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('Velocity in East ($\frac{m}{s}$)', 'interpreter', 'latex')
grid on
grid minor
range_ve = max(Ve) - min(Ve);
buffer_ve = max(0.2, 0.05 * range_ve);
ylim([min(Ve) - buffer_ve, max(Ve) + buffer_ve])

% --- Down velocity ---
subplot(3,1,3)
plot(t, Vd, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('Velocity in Down ($\frac{m}{s}$)', 'interpreter', 'latex')
grid on
grid minor
range_vd = max(Vd) - min(Vd);
buffer_vd = max(0.2, 0.05 * range_vd);
ylim([min(Vd) - buffer_vd, max(Vd) + buffer_vd])

% --- Auto-save ---
if export_fig == 1
    exportgraphics(fig, 'Velocity in initial frame.png', 'BackgroundColor', 'none', 'ContentType', 'image')
end
%% Velocity in body frame
fig = figure('Name','Velocity in body frame');

% --- u_b (forward velocity) ---
subplot(3,1,1)
plot(t, X1, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$u_b$ ($\frac{m}{s}$)', 'interpreter', 'latex')
grid on; grid minor

u_max = max(X1);
u_min = min(X1);
range_u = u_max - u_min;
if range_u < 0.05
    mid_val = (u_max + u_min) / 2;
    ylim([mid_val - 0.05, mid_val + 0.05])
else
    u_buffer = 0.05 * range_u;
    ylim([u_min - u_buffer, u_max + u_buffer])
end

% --- v_b (side velocity) ---
subplot(3,1,2)
plot(t, X2, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$v_b$ ($\frac{m}{s}$)', 'interpreter', 'latex')
grid on; grid minor

v_peak = max(abs(X2));
if v_peak < 0.05
    ylim([-0.1, 0.1])
else
    v_buffer = 0.1 * v_peak;
    ylim([-v_peak - v_buffer, v_peak + v_buffer])
end

% --- w_b (vertical velocity) ---
subplot(3,1,3)
plot(t, X3, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$w_b$ ($\frac{m}{s}$)', 'interpreter', 'latex')
grid on; grid minor

w_peak = max(abs(X3));
if w_peak < 0.05
    ylim([-0.1, 0.1])
else
    w_buffer = 0.1 * w_peak;
    ylim([-w_peak - w_buffer, w_peak + w_buffer])
end

% --- Auto-save ---
if export_fig == 1
    exportgraphics(fig, 'Velocity in body frame.png', 'BackgroundColor', 'none', 'ContentType', 'image')
end


%% Velocity in initial frame Vs Velocity in body frame
fig = figure('Name','Velocity in initial frame Vs Velocity in body frame');

% --- North vs u (body x-axis) ---
subplot(3,1,1)
plot(t, Vn, 'Color', 'red', 'LineWidth', 1.5); hold on
plot(t, X1, 'Color', 'blue', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('Velocity ($\frac{m}{s}$)', 'interpreter', 'latex')
legend('$V_n$', '$u_b$', 'interpreter', 'latex')
grid on; grid minor

v1_min = min([Vn(:); X1(:)]);
v1_max = max([Vn(:); X1(:)]);
v1_range = v1_max - v1_min;
buffer = max(0.1, 0.05 * v1_range);  % minimum buffer
ylim([v1_min - buffer, v1_max + buffer])
hold off

% --- East vs v (body y-axis) ---
subplot(3,1,2)
plot(t, Ve, 'Color', 'red', 'LineWidth', 1.5); hold on
plot(t, X2, 'Color', 'blue', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('Velocity ($\frac{m}{s}$)', 'interpreter', 'latex')
legend('$V_e$', '$v_b$', 'interpreter', 'latex')
grid on; grid minor

v2_min = min([Ve(:); X2(:)]);
v2_max = max([Ve(:); X2(:)]);
v2_range = v2_max - v2_min;
buffer = max(0.1, 0.05 * v2_range);
ylim([v2_min - buffer, v2_max + buffer])
hold off

% --- Down vs w (body z-axis) ---
subplot(3,1,3)
plot(t, Vd, 'Color', 'red', 'LineWidth', 1.5); hold on
plot(t, X3, 'Color', 'blue', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('Velocity ($\frac{m}{s}$)', 'interpreter', 'latex')
legend('$V_d$', '$w_b$', 'interpreter', 'latex')
grid on; grid minor

v3_min = min([Vd(:); X3(:)]);
v3_max = max([Vd(:); X3(:)]);
v3_range = v3_max - v3_min;
buffer = max(0.1, 0.05 * v3_range);
ylim([v3_min - buffer, v3_max + buffer])
hold off

% --- Save figure if export flag is enabled ---
if export_fig == 1
    exportgraphics(fig, 'Velocity in initial frame Vs Velocity in body frame.png', ...
        'BackgroundColor', 'none', 'ContentType', 'image')
end

%--- Air Data ---
% Longitudinal
fig = figure('Name', 'Air data longitudinal');

% --- u (forward velocity) ---
subplot(3,1,1)
plot(t, X1, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$u$ ($\frac{m}{s}$)', 'interpreter', 'latex')
grid on; grid minor

u_min = min(X1);
u_max = max(X1);
u_range = u_max - u_min;
u_buffer = max(0.5, 0.05 * u_range);  % Ensure minimum visual spacing
ylim([u_min - u_buffer, u_max + u_buffer])

% --- w (vertical velocity) ---
subplot(3,1,2)
plot(t, X3, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$w$ ($\frac{m}{s}$)', 'interpreter', 'latex')
grid on; grid minor

w_peak = max(abs(X3));
if w_peak < 0.05
    ylim([-0.1 0.1])  % Constant value fallback
else
    w_buffer = max(0.2, 0.1 * w_peak);
    ylim([-w_peak - w_buffer, w_peak + w_buffer])
end

% --- alpha (angle of attack) ---
subplot(3,1,3)
alpha_deg = alpha * 180 / pi;
plot(t, alpha_deg, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$\alpha$ (deg)', 'interpreter', 'latex')
grid on; grid minor

alpha_max = max(alpha_deg);
alpha_min = min(alpha_deg);
alpha_peak = max(abs(alpha_deg));
alpha_range = alpha_max - alpha_min;
alpha_buffer = max(0.5, 0.1 * alpha_range);

if alpha_peak > 1
    ylim([alpha_min - alpha_buffer, alpha_max + alpha_buffer])
else
    ylim([-2 2])
end

% --- Export ---
if export_fig == 1
    exportgraphics(fig, 'Air data in longitudinal.png', 'BackgroundColor', 'none', 'ContentType', 'image')
end

%% Lateral
fig = figure('Name','Air data lateral');

% --- Airspeed Va ---
subplot(3,1,1)
plot(t, Va, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$V_a$ ($\frac{m}{s}$)', 'interpreter', 'latex')
grid on; grid minor

Va_min = max(0, min(Va));  % Ensure no negative airspeed
Va_max = max(Va);
Va_range = Va_max - Va_min;
Va_buffer = max(0.5, 0.05 * Va_range);  % Minimum buffer of 0.5 m/s
ylim([Va_min - Va_buffer, Va_max + Va_buffer])

% --- v (lateral velocity) ---
subplot(3,1,2)
plot(t, X2, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$v$ ($\frac{m}{s}$)', 'interpreter', 'latex')
grid on; grid minor

v_peak = max(abs(X2));
if v_peak < 0.05
    ylim([-0.1 0.1])  % Fallback for near-zero values
else
    v_buffer = max(0.2, 0.1 * v_peak);  % Minimum buffer
    ylim([-v_peak - v_buffer, v_peak + v_buffer])
end

% --- beta (sideslip angle) ---
subplot(3,1,3)
beta_deg = beta * 180 / pi;
plot(t, beta_deg, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$\beta$ (deg)', 'interpreter', 'latex')
grid on; grid minor

beta_max = max(beta_deg);
beta_min = min(beta_deg);
beta_peak = max(abs(beta_deg));
beta_range = beta_max - beta_min;
beta_buffer = max(0.5, 0.1 * beta_range);  % Minimum buffer

if beta_peak > 1
    ylim([beta_min - beta_buffer, beta_max + beta_buffer])
else
    ylim([-2 2])
end

% --- Export ---
if export_fig == 1
    exportgraphics(fig, 'Air data in lateral.png', 'BackgroundColor', 'none', 'ContentType', 'image')
end
%% states
% Euler angles
fig = figure('Name', 'Euler angles');

% --- Roll (phi) ---
subplot(3,1,1)
phi_deg = X4 * 180 / pi;
plot(t, phi_deg, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$\phi$ (deg)', 'interpreter', 'latex')
grid on; grid minor

phi_max = max(phi_deg);
phi_min = min(phi_deg);
phi_range = phi_max - phi_min;
phi_buffer = max(2, 0.1 * phi_range);  % Minimum 2 deg buffer

if phi_range < 1
    ylim([-5 5])  % Fallback
else
    ylim([phi_min - phi_buffer, phi_max + phi_buffer])
end

% --- Pitch (theta) ---
subplot(3,1,2)
theta_deg = X5 * 180 / pi;
plot(t, theta_deg, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$\theta$ (deg)', 'interpreter', 'latex')
grid on; grid minor

theta_max = max(theta_deg);
theta_min = min(theta_deg);
theta_range = theta_max - theta_min;
theta_buffer = max(2, 0.1 * theta_range);

if theta_range < 1
    ylim([-5 5])
else
    ylim([theta_min - theta_buffer, theta_max + theta_buffer])
end

% --- Yaw (psi) ---
subplot(3,1,3)
psi_deg = X6 * 180 / pi;
plot(t, psi_deg, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$\psi$ (deg)', 'interpreter', 'latex')
grid on; grid minor

psi_max = max(psi_deg);
psi_min = min(psi_deg);
psi_range = psi_max - psi_min;

if max(abs(psi_deg)) > 90
    ylim([-180, 180])
elseif psi_range < 1
    ylim([-5 5])
else
    psi_buffer = max(2, 0.1 * psi_range);
    ylim([psi_min - psi_buffer, psi_max + psi_buffer])
end

% --- Export ---
if export_fig == 1
    exportgraphics(fig, 'Euler Angles.png', 'BackgroundColor', 'none', 'ContentType', 'image')
end

%% Angular rates
fig = figure('Name','Angular rates');

% --- Roll rate (p) ---
subplot(3,1,1)
p_deg = X7 * 180 / pi;
plot(t, p_deg, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('p ($\frac{deg}{s}$)', 'interpreter', 'latex')
grid on; grid minor

p_max = max(p_deg);
p_min = min(p_deg);
p_range = p_max - p_min;
p_buffer = max(5, 0.1 * p_range);
if p_range < 1
    ylim([-10 10])
else
    ylim([p_min - p_buffer, p_max + p_buffer])
end

% --- Pitch rate (q) ---
subplot(3,1,2)
q_deg = X8 * 180 / pi;
plot(t, q_deg, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('q ($\frac{deg}{s}$)', 'interpreter', 'latex')
grid on; grid minor

q_max = max(q_deg);
q_min = min(q_deg);
q_range = q_max - q_min;
q_buffer = max(5, 0.1 * q_range);
if q_range < 1
    ylim([-10 10])
else
    ylim([q_min - q_buffer, q_max + q_buffer])
end

% --- Yaw rate (r) ---
subplot(3,1,3)
r_deg = X9 * 180 / pi;
plot(t, r_deg, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('r ($\frac{deg}{s}$)', 'interpreter', 'latex')
grid on; grid minor

r_max = max(r_deg);
r_min = min(r_deg);
r_range = r_max - r_min;
r_buffer = max(5, 0.1 * r_range);
if r_range < 1
    ylim([-10 10])
else
    ylim([r_min - r_buffer, r_max + r_buffer])
end

% --- Export ---
if export_fig == 1
    exportgraphics(fig, 'Angular rates.png', 'BackgroundColor', 'none', 'ContentType', 'image')
end

%% Forces and Moments
% Forces
fig = figure('Name','Forces');

% --- Fx (Longitudinal force) ---
subplot(3,1,1)
plot(t, Fx, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$F_X$ (N)', 'interpreter', 'latex')
grid on; grid minor

Fx_max = max(Fx); Fx_min = min(Fx);
Fx_range = Fx_max - Fx_min;
Fx_buffer = max(2, 0.1 * Fx_range);
if Fx_range < 1
    ylim([Fx_min - 5, Fx_max + 5])
else
    ylim([Fx_min - Fx_buffer, Fx_max + Fx_buffer])
end

% --- Fy (Lateral force) ---
subplot(3,1,2)
plot(t, Fy, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$F_Y$ (N)', 'interpreter', 'latex')
grid on; grid minor

Fy_max = max(Fy); Fy_min = min(Fy);
Fy_range = Fy_max - Fy_min;
Fy_buffer = max(2, 0.2 * Fy_range);
if Fy_range < 1
    ylim([Fy_min - 5, Fy_max + 5])
else
    ylim([Fy_min - Fy_buffer, Fy_max + Fy_buffer])
end

% --- Fz (Vertical force) ---
subplot(3,1,3)
plot(t, -Fz, 'Color', 'red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$F_Z$ (N)', 'interpreter', 'latex')
grid on; grid minor

Fz_max = max(-Fz); Fz_min = min(-Fz);
Fz_range = Fz_max - Fz_min;
Fz_buffer = max(2, 0.2 * Fz_range);
if Fz_range < 1
    ylim([Fz_min - 5, Fz_max + 5])
else
    ylim([Fz_min - Fz_buffer, Fz_max + Fz_buffer])
end

% Export figure
if export_fig == 1
    exportgraphics(fig, 'Forces.png', 'BackgroundColor', 'none', 'ContentType', 'image')
end
%% Moments
fig = figure('Name','Moments');

% --- Roll moment (l) ---
subplot(3,1,1)
plot(t, l, 'Color','red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('l (N.m)', 'interpreter','latex')
grid on; grid minor
l_max = max(l); l_min = min(l);
l_range = l_max - l_min;
l_buffer = max(10, 0.1 * l_range);
if l_range < 5
    ylim([-50 50])
else
    ylim([l_min - l_buffer, l_max + l_buffer])
end

% --- Pitch moment (M) ---
subplot(3,1,2)
plot(t, M, 'Color','red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('M (N.m)', 'interpreter','latex')
grid on; grid minor
M_max = max(M); M_min = min(M);
M_range = M_max - M_min;
M_buffer = max(10, 0.1 * M_range);
if M_range < 5
    ylim([-50 50])
else
    ylim([M_min - M_buffer, M_max + M_buffer])
end

% --- Yaw moment (N) ---
subplot(3,1,3)
plot(t, N, 'Color','red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('N (N.m)', 'interpreter','latex')
grid on; grid minor
N_max = max(N); N_min = min(N);
N_range = N_max - N_min;
N_buffer = max(10, 0.1 * N_range);
if N_range < 5
    ylim([-50 50])
else
    ylim([N_min - N_buffer, N_max + N_buffer])
end

% Export the figure if requested
if export_fig == 1
    exportgraphics(fig, 'Momenta.png', 'BackgroundColor', 'none', 'ContentType', 'image')
end

%% Control surfaces
fig = figure('Name','Control surfaces deflection');

% --- Elevator deflection ---
subplot(4,1,1)
plot(t, delta_e * 180/pi, 'Color','red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$\delta_{e}$ (deg)','interpreter','latex')
ylim([-10 10])
grid on; grid minor

% --- Aileron deflection ---
subplot(4,1,2)
plot(t, delta_a * 180/pi, 'Color','red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$\delta_{a}$ (deg)','interpreter','latex')
ylim([-10 10])
grid on; grid minor

% --- Rudder deflection ---
subplot(4,1,3)
plot(t, delta_r * 180/pi, 'Color','red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$\delta_{r}$ (deg)','interpreter','latex')
ylim([-10 10])
grid on; grid minor

% --- Throttle input ---
subplot(4,1,4)
plot(t, delta_t, 'Color','red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$\delta_{th}$','interpreter','latex')
ylim([0 1])
grid on; grid minor

% Export the figure if requested
if export_fig2 == 1
    exportgraphics(fig, 'Control surfaces deflection.png', 'BackgroundColor', 'none', 'ContentType', 'image')
end
%% Euler angles and angular rates in one figure
fig = figure('Name','Euler angles and Angular rates');

% Convert to degrees
phi_deg   = X4   * 180/pi;
theta_deg = X5 * 180/pi;
psi_deg   = X6   * 180/pi;
p_deg = X7 * 180/pi;
q_deg = X8 * 180/pi;
r_deg = X9 * 180/pi;

% Define subplot positions for clarity
subplot_order = {
    {1, phi_deg,    '$\phi$ (deg)',     [-5 5], 5, 0.1};
    {3, theta_deg,  '$\theta$ (deg)',   [-5 5],   5, 0.1};
    {5, psi_deg,    '$\psi$ (deg)',     [-5 5], 5, 0.1};
    {2, p_deg,      '$p$ ($\frac{deg}{s}$)', [-10 10], 5, 0.1};
    {4, q_deg,      '$q$ ($\frac{deg}{s}$)', [-10 10], 5, 0.1};
    {6, r_deg,      '$r$ ($\frac{deg}{s}$)', [-10 10], 5, 0.1};
};

% Plot each one
for k = 1:size(subplot_order,1)
    idx = subplot_order{k}{1};
    data = subplot_order{k}{2};
    label_str = subplot_order{k}{3};
    fallback_ylim = subplot_order{k}{4};
    buffer_min = subplot_order{k}{5};
    buffer_pct = subplot_order{k}{6};

    subplot(3,2,idx)
    plot(t, data, 'Color', 'red', 'LineWidth', 1.5)
    xlabel('time (sec)')
    ylabel(label_str, 'interpreter','latex')
    grid on; grid minor

    xlim([0 50])
    d_max = max(data); d_min = min(data);
    d_range = d_max - d_min;
    d_buffer = max(buffer_min, buffer_pct * d_range);
    if d_range < 1
        ylim(fallback_ylim)
    else
        ylim([d_min - d_buffer, d_max + d_buffer])
    end
end

%sgtitle('Euler Angles and Angular Rates', 'FontWeight', 'bold')

% Export if enabled
if export_fig == 1
    exportgraphics(fig, 'Euler and Angular Rates.png', 'BackgroundColor','none','ContentType','image')
end

%% Body-frame Velocities and Position in NED
fig = figure('Name', 'Body Velocities and Position');

% Define subplot data and labels
subplot_data = {
    {1, X1, '$u$ ($\frac{m}{s}$)', 0.5, 0.05};  % Forward
    {3, X2, '$v$ ($\frac{m}{s}$)', 0.5, 0.1};   % Lateral
    {5, X3, '$w$ ($\frac{m}{s}$)', 0.5, 0.1};   % Vertical
    {2, Pn, '$x$ (m)', 2, 0.05};               % NED X
    {4, Pe, '$y$ (m)', 2, 0.05};               % NED Y
    {6, -Pd, '$z$ (m)', 2, 0.05};               % NED Z
};

% Plot
for k = 1:6
    idx = subplot_data{k}{1};
    data = subplot_data{k}{2};
    label_str = subplot_data{k}{3};
    buffer_min = subplot_data{k}{4};
    buffer_pct = subplot_data{k}{5};

    subplot(3,2,idx)
    plot(t, data, 'Color', 'red', 'LineWidth', 1.5)
    xlabel('time (sec)')
    ylabel(label_str, 'Interpreter','latex')
    grid on; grid minor
    xlim([0 max(t)])

    d_max = max(data); d_min = min(data);
    d_range = d_max - d_min;
    d_buffer = max(buffer_min, buffer_pct * d_range);

    if d_range < 1
        ylim([d_min - 2, d_max + 2])
    else
        ylim([d_min - d_buffer, d_max + d_buffer])
    end
end

%sgtitle('Body-frame Velocities and NED Position', 'FontWeight', 'bold')

% Export if enabled
if export_fig == 1
    exportgraphics(fig, 'Velocity and Position.png', 'BackgroundColor','none','ContentType','image')
end

%% Angle of Attack (alpha) and Sideslip Angle (beta)
fig = figure('Name','Alpha and Beta');

% Calculate angles in degrees
V_total = sqrt(X1.^2 + X2.^2 + X3.^2);
alpha = atan2(X3, X1) * 180/pi;              % angle of attack (deg)
beta  = asin(X2 ./ V_total) * 180/pi;       % sideslip angle (deg)

% --- Alpha ---
subplot(2,1,1)
plot(t, alpha, 'Color','red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$\alpha$ (deg)', 'Interpreter','latex')
grid on; grid minor
xlim([0 50])
ylim([-10 10])
% alpha_max = max(alpha); alpha_min = min(alpha);
% alpha_range = max(abs(alpha_max), abs(alpha_min));  % max abs value to center around 0
% alpha_buffer = max(2, 0.1 * alpha_range);
% ylim([-1 1] * (alpha_range + alpha_buffer));  % Symmetric about zero


% --- Beta ---
subplot(2,1,2)
plot(t, beta, 'Color','red', 'LineWidth', 1.5)
xlabel('time (sec)')
ylabel('$\beta$ (deg)', 'Interpreter','latex')
grid on; grid minor
xlim([0 50])
ylim([-10 10])
% beta_max = max(beta); beta_min = min(beta);
% beta_range = beta_max - beta_min;
% beta_buffer = max(2, 0.1 * beta_range);
% if beta_range < 1
%     ylim([-10 10])
% else
%     ylim([beta_min - beta_buffer, beta_max + beta_buffer])
% end

%sgtitle('Angle of Attack and Sideslip Angle', 'FontWeight', 'normal ')

% Export if enabled
if export_fig == 1
    exportgraphics(fig, 'Alpha_Beta.png', 'BackgroundColor','none','ContentType','image')
end

%% Forces and Moments - Combined Figure
fig = figure('Name','Forces and Moments');

% Subplot order and settings
subplot_data = {
    1, Fx, '$F_X$ (N)', 2, 0.1;
    3, Fy, '$F_Y$ (N)', 2, 0.2;
    5, -Fz, '$F_Z$ (N)', 2, 0.2;
    2, l,  '$l$ (N$\cdot$m)', 10, 0.1;
    4, M,  '$M$ (N$\cdot$m)', 10, 0.1;
    6, N,  '$N$ (N$\cdot$m)', 10, 0.1;
};

for k = 1:size(subplot_data,1)
    idx = subplot_data{k,1};
    data = subplot_data{k,2};
    label_str = subplot_data{k,3};
    buffer_min = subplot_data{k,4};
    buffer_pct = subplot_data{k,5};

    subplot(3,2,idx)
    plot(t, data, 'Color','red', 'LineWidth', 1.5)
    xlabel('time (sec)')
    ylabel(label_str, 'interpreter','latex')
    grid on; grid minor
    xlim([0 50])

    % Improved dynamic Y-limits
    d_max = max(data); d_min = min(data);
    d_range = d_max - d_min;
    if d_range < 1e-3
        center = (d_max + d_min) / 2;
        ylim([center - 0.05, center + 0.05]);  % Tune if needed
    else
        d_buffer = max(buffer_min, buffer_pct * d_range);
        ylim([d_min - d_buffer, d_max + d_buffer])
    end
end

%sgtitle('Aerodynamic Forces and Moments', 'FontWeight', 'bold')

% Export the figure if enabled
if export_fig == 1
    exportgraphics(fig, 'Forces_and_Moments.png', 'BackgroundColor','none','ContentType','image')
end
