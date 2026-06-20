clc; clear; close all;
%% =========================================================
%                LOAD TRACK CSV
% =========================================================
race = readmatrix("Monza.csv");
path_x = race(2:end,1);
path_y = race(2:end,2);

%% =========================================================
%                VEHICLE PARAMETERS
% =========================================================
L_f = 1.535*0.51; L_r =1.535*0.49; L = 1.535;
Mass = 270; Iz = 47;
Cf = 35238.459; Cr = 35033.582;
dt = 0.02; mu = 0.7;
v_max = 40; v_switch = 5.0; 

%% =========================================================
%                HYBRID CONTROLLER GAINS
% =========================================================
% Stanley Gains
k_stanley = 1.5;    
v_eps = 1.0;
% Pure Pursuit Gains (Look-ahead)
k_pure = 0.8;       % Look-ahead speed gain
L_min  = 3.0;        % Minimum look-ahead distance
% Longitudinal PID
Kp_v = 1.5; Ki_v = 0.2;

%% =========================================================
%        ARC-LENGTH REPARAMETERIZATION
% =========================================================
dx = diff(path_x); dy = diff(path_y);
ds = hypot(dx,dy);
s = [0; cumsum(ds)];
valid = [true; ds > 1e-3];
s = s(valid); x = path_x(valid); y = path_y(valid);
s_uniform = linspace(0, s(end), length(s));
path_x = interp1(s,x,s_uniform,'pchip')';
path_y = interp1(s,y,s_uniform,'pchip')';
Npath = length(path_x);

% Curvature-based Speed Profile
kappa_track = zeros(Npath,1);
for i = 2:Npath-1
    psi1 = atan2(path_y(i)-path_y(i-1), path_x(i)-path_x(i-1));
    psi2 = atan2(path_y(i+1)-path_y(i), path_x(i+1)-path_x(i));
    kappa_track(i) = atan2(sin(psi2-psi1), cos(psi2-psi1)) / max(hypot(path_x(i+1)-path_x(i), path_y(i+1)-path_y(i)), 1e-3);
end
v_race = min(v_max, sqrt((mu * 9.81) ./ max(abs(kappa_track), 1e-3)));

%% =========================================================
%                INITIAL STATES & PLOT SETUP
% =========================================================
X = path_x(1); Y = path_y(1); PSI = atan2(path_y(2)-path_y(1), path_x(2)-path_x(1));
vx = 5.0; vy = 0; r = 0; 
idx_closest = 1; vel_int = 0;

fig = figure('Name','Hybrid Controller: Stanley + Pure Pursuit','Color','k','Units','normalized','Position',[0.05 0.05 0.9 0.85]);

% Main Map
ax1 = subplot(4,4,[1 2 5 6 9 10 13 14]); 
plot(ax1, path_x, path_y, 'w--', 'LineWidth', 1); hold on;
h_trail = plot(ax1, X, Y, 'c-', 'LineWidth', 1.5);
h_car   = plot(ax1, X, Y, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
h_ld    = plot(ax1, X, Y, 'gp', 'MarkerSize', 10, 'LineWidth', 1.5); % Look-ahead point
title(ax1, 'Hybrid Logic Track Trace', 'Color', 'w');
axis(ax1, 'equal'); grid(ax1, 'on'); set(ax1, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');

% Telemetry Plots
ax2 = subplot(4,4,[3 4]); h_vel = animatedline(ax2, 'Color', 'g', 'LineWidth', 2);
ylabel(ax2, 'Speed [m/s]', 'Color', 'w'); grid on; set(ax2, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');

ax3 = subplot(4,4,[7 8]); h_delta = animatedline(ax3, 'Color', 'm', 'LineWidth', 1.5);
ylabel(ax3, 'Delta [deg]', 'Color', 'w'); grid on; set(ax3, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');

ax4 = subplot(4,4,[11 12]); h_err = animatedline(ax4, 'Color', 'y', 'LineWidth', 1.5);
ylabel(ax4, 'CTE [m]', 'Color', 'w'); grid on; title(ax4, 'Cross Track Error', 'Color', 'w');
set(ax4, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');

h_text = annotation('textbox', [0.75, 0.05, 0.2, 0.05], 'String', 'Init...', 'Color', 'w', 'EdgeColor', 'y');

%% =========================================================
%                MAIN SIMULATION LOOP
% =========================================================
traj_x = []; traj_y = [];
for k = 1:150000
    % --- 1. SENSING (Pure Pursuit Concept) ---
    xf = X + L_f * cos(PSI); yf = Y + L_f * sin(PSI); % Front Axle
    
    % Dynamic Look-ahead Distance
    L_ld = k_pure * vx + L_min; 
    
    % Find Closest Point to determine search window
    idx_end = min(idx_closest+200, Npath);
    [~,i_m] = min((path_x(idx_closest:idx_end)-xf).^2 + (path_y(idx_closest:idx_end)-yf).^2);
    idx_closest = idx_closest + i_m - 1;
    
    % Find Look-ahead Point (Pure Pursuit logic)
    idx_ld = idx_closest;
    while idx_ld < Npath && norm([path_x(idx_ld)-xf, path_y(idx_ld)-yf]) < L_ld
        idx_ld = idx_ld + 1;
    end
    target_x = path_x(idx_ld); target_y = path_y(idx_ld);

    % --- 2. CONTROL (Stanley Concept) ---
    % Path heading at look-ahead point
    if idx_ld < Npath
        path_psi = atan2(path_y(idx_ld+1)-path_y(idx_ld), path_x(idx_ld+1)-path_x(idx_ld));
    else
        path_psi = atan2(path_y(idx_ld)-path_y(idx_ld-1), path_x(idx_ld)-path_x(idx_ld-1));
    end
    
    % Cross-track Error at Look-ahead point
    e_ct = (target_y - yf)*cos(path_psi) - (target_x - xf)*sin(path_psi);
    theta_e = atan2(sin(path_psi - PSI), cos(path_psi - PSI));
    
    % Combined Control Law
    delta = theta_e + atan2(k_stanley * e_ct, vx + v_eps);
    delta = max(min(delta, deg2rad(25)), -deg2rad(25));
    
    % Speed Control
    v_ref = v_race(idx_closest);
    v_err = v_ref - vx; vel_int = vel_int + v_err*dt;
    acc = Kp_v*v_err + Ki_v*vel_int;

    % --- 3. PHYSICS SWITCHING ---
    if vx < v_switch
        beta = atan((L_r / L) * tan(delta));
        dX = vx * cos(PSI + beta); dY = vx * sin(PSI + beta); dPSI = (vx / L_r) * sin(beta);
        dvx = acc; dvy = 0; dr = 0;
        model_str = 'KINEMATIC + HYBRID CONTROL';
    else
        alpha_f = delta - atan2((vy + L_f*r), vx);
        alpha_r = -atan2((vy - L_r*r), vx);
        Fyf = Cf * alpha_f; Fyr = Cr * alpha_r;
        dX = vx*cos(PSI) - vy*sin(PSI); dY = vx*sin(PSI) + vy*cos(PSI);
        dPSI = r; dvx = acc;
        dvy = (Fyf + Fyr) / Mass - vx*r;
        dr = (L_f*Fyf - L_r*Fyr) / Iz;
        model_str = 'DYNAMIC + HYBRID CONTROL';
    end

    % --- 4. INTEGRATION ---
    X = X + dX*dt; Y = Y + dY*dt; PSI = PSI + dPSI*dt;
    vx = max(vx + dvx*dt, 0.1); vy = vy + dvy*dt; r = r + dr*dt;

    % --- 5. LIVE PLOTTING ---
    traj_x(end+1) = X; traj_y(end+1) = Y;
    if mod(k,10) == 0
        curr_t = k * dt;
        addpoints(h_vel, curr_t, vx);
        addpoints(h_delta, curr_t, rad2deg(delta));
        addpoints(h_err, curr_t, e_ct);
        
        set(h_car, 'XData', X, 'YData', Y);
        set(h_ld, 'XData', target_x, 'YData', target_y);
        set(h_trail, 'XData', traj_x, 'YData', traj_y);
        set(h_text, 'String', model_str);
        
        xlim(ax2, [max(0, curr_t-5) curr_t+1]);
        xlim(ax3, [max(0, curr_t-5) curr_t+1]);
        xlim(ax4, [max(0, curr_t-5) curr_t+1]);
        drawnow limitrate;
    end
    
    if idx_closest >= Npath-5, break; end
end