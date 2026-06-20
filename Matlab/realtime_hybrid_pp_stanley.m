clc; clear; close all;
%% =========================================================
%                LOAD TRACK CSV
% =========================================================
% Ensure this file exists in your directory
race = readmatrix("traj_race_cl.csv");
path_x_raw = race(4:end,2);
path_y_raw = race(4:end,3);

%% =========================================================
%                VEHICLE PARAMETERS
% =========================================================
L_f = 1.535*0.51; L_r = 1.535*0.49; L = 1.535;
Mass = 270; Iz = 47;
Cf = 35238.459; Cr = 35033.582;
dt = 0.01; mu = 0.7;
v_max = 40; v_switch = 5.0; 

%% =========================================================
%                THROTTLE MAPPING CONSTANTS
% =========================================================
max_throttle_rad = pi/2;      % 90 degrees in radians (max butterfly valve opening)
max_acc_capability = mu*9.81; % Theoretical max acceleration based on friction [cite: 10]

%% =========================================================
%                HYBRID CONTROLLER GAINS
% =========================================================
k_stanley = 2.5; v_eps = 1.0;
k_pure = 0.15; L_min = 1.5; 
Kp_v = 1.5; Ki_v = 0.2;

%% =========================================================
%        ARC-LENGTH REPARAMETERIZATION
% =========================================================
dx = diff(path_x_raw); dy = diff(path_y_raw);
ds = hypot(dx,dy);
s_raw = [0; cumsum(ds)];
valid = [true; ds > 1e-3];
s_valid = s_raw(valid); x_valid = path_x_raw(valid); y_valid = path_y_raw(valid);
s_uniform = 0:0.1:s_valid(end);
path_x = interp1(s_valid, x_valid, s_uniform, 'pchip')';
path_y = interp1(s_valid, y_valid, s_uniform, 'pchip')';
Npath = length(path_x);

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
X = path_x(1); Y = path_y(1); 
PSI = atan2(path_y(2)-path_y(1), path_x(2)-path_x(1));
vx = 5.0; vy = 0; r = 0; 
idx_closest = 1; vel_int = 0;

fig = figure('Name','Real-Time Hybrid Controller','Color','k','Units','normalized','Position',[0.05 0.05 0.9 0.85]);
ax1 = subplot(4,4,[1 2 5 6 9 10 13 14]); 
plot(ax1, path_x, path_y, 'w--', 'LineWidth', 1); hold on;
h_trail = plot(ax1, X, Y, 'c-', 'LineWidth', 1.5);
h_car   = plot(ax1, X, Y, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
h_ld    = plot(ax1, X, Y, 'gp', 'MarkerSize', 10, 'LineWidth', 1.5);
axis(ax1, 'equal'); grid(ax1, 'on'); set(ax1, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');
ax2 = subplot(4,4,[3 4]); h_vel = animatedline(ax2, 'Color', 'g', 'LineWidth', 2);
ylabel(ax2, 'Speed [m/s]', 'Color', 'w'); grid on; set(ax2, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');
ax3 = subplot(4,4,[7 8]); h_delta = animatedline(ax3, 'Color', 'm', 'LineWidth', 1.5);
ylabel(ax3, 'Delta [deg]', 'Color', 'w'); grid on; set(ax3, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');
ax4 = subplot(4,4,[11 12]); h_err = animatedline(ax4, 'Color', 'y', 'LineWidth', 1.5);
ylabel(ax4, 'CTE [m]', 'Color', 'w'); grid on; set(ax4, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');
h_timer = annotation('textbox', [0.05, 0.05, 0.15, 0.05], 'String', 'Time: 0.00s', 'Color', 'g', 'FontSize', 12, 'BackgroundColor', 'k');

%% =========================================================
%                MAIN SIMULATION LOOP
% =========================================================
traj_x = []; traj_y = [];
steer_history = [];    
throttle_history = []; % Original acceleration (m/s^2)
throttle_rad_history = []; % NEW: Throttle input in radians

fprintf('Starting Real-Time Simulation...\n');
tic; 
for k = 1:150000
    % --- 1. SENSING ---
    xf = X + L_f * cos(PSI); yf = Y + L_f * sin(PSI);
    idx_end = min(idx_closest + 300, Npath);
    [~, i_m] = min((path_x(idx_closest:idx_end)-xf).^2 + (path_y(idx_closest:idx_end)-yf).^2);
    idx_closest = idx_closest + i_m - 1;
    
    L_ld = k_pure * vx + L_min; 
    idx_ld = idx_closest;
    while idx_ld < Npath && norm([path_x(idx_ld)-xf, path_y(idx_ld)-yf]) < L_ld
        idx_ld = idx_ld + 1;
    end
    target_x = path_x(idx_ld); target_y = path_y(idx_ld);
    
    % --- 2. CONTROL LAW ---
    if idx_closest < Npath
        path_psi = atan2(path_y(idx_closest+1)-path_y(idx_closest), path_x(idx_closest+1)-path_x(idx_closest));
    else
        path_psi = atan2(path_y(idx_closest)-path_y(idx_closest-1), path_x(idx_closest)-path_x(idx_closest-1));
    end
    
    e_ct = (path_y(idx_closest) - yf)*cos(path_psi) - (path_x(idx_closest) - xf)*sin(path_psi);
    theta_e = atan2(sin(path_psi - PSI), cos(path_psi - PSI));
    
    beta_comp = 0;
    if vx >= v_switch
        beta_comp = atan2(vy, vx); 
    end
    
    delta = (theta_e + beta_comp) + atan2(k_stanley * e_ct, vx + v_eps);
    delta = max(min(delta, deg2rad(25)), -deg2rad(25));
    
    % Speed/Throttle Control
    v_ref = v_race(idx_closest);
    v_err = v_ref - vx; vel_int = vel_int + v_err*dt;
    acc = Kp_v*v_err + Ki_v*vel_int;

    % --- 3. PHYSICS ---
    if vx < v_switch
        beta = atan((L_r / L) * tan(delta));
        dX = vx * cos(PSI + beta); dY = vx * sin(PSI + beta); dPSI = (vx / L_r) * sin(beta);
        dvx = acc; dvy = 0; dr = 0;
    else
        alpha_f = delta - atan2((vy + L_f*r), vx);
        alpha_r = -atan2((vy - L_r*r), vx);
        Fyf = Cf * alpha_f; Fyr = Cr * alpha_r;
        dX = vx*cos(PSI) - vy*sin(PSI); dY = vx*sin(PSI) + vy*cos(PSI);
        dPSI = r; dvx = acc; dvy = (Fyf + Fyr) / Mass - vx*r; dr = (L_f*Fyf - L_r*Fyr) / Iz;
    end
    
    % --- 4. THROTTLE TO RADIANS MAPPING ---
    % Map positive acceleration (throttle) to a 0 to pi/2 range
    % Negative acceleration (braking) is treated as 0 throttle
    throttle_rad = max(0, (acc / max_acc_capability)) * max_throttle_rad;
    throttle_rad = min(throttle_rad, max_throttle_rad); % Clamp to physical max

    % --- 5. INTEGRATION & STORAGE ---
    X = X + dX*dt; Y = Y + dY*dt; PSI = PSI + dPSI*dt;
    vx = max(vx + dvx*dt, 0.1); vy = vy + dvy*dt; r = r + dr*dt;

    traj_x(end+1) = X; 
    traj_y(end+1) = Y;
    steer_history(end+1) = delta;      
    throttle_history(end+1) = acc;     
    throttle_rad_history(end+1) = throttle_rad; % Store throttle angle

    % --- 6. LIVE PLOTTING ---
    if mod(k,30) == 0 
        curr_t = k * dt;
        addpoints(h_vel, curr_t, vx);
        addpoints(h_delta, curr_t, rad2deg(delta));
        addpoints(h_err, curr_t, e_ct);
        set(h_car, 'XData', X, 'YData', Y);
        set(h_ld, 'XData', target_x, 'YData', target_y);
        set(h_trail, 'XData', traj_x, 'YData', traj_y);
        set(h_timer, 'String', sprintf('Time: %.2fs', curr_t));
        drawnow limitrate;
    end
    
    while toc < k * dt; end
    if idx_closest >= Npath-5, break; end
end
fprintf('Lap Completed \n');