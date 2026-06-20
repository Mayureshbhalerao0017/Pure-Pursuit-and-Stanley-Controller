clc; clear; close all;

%% =========================================================
%                LOAD TRACK CSV
% =========================================================
% Note: Ensure YasMarina.csv is in the correct path
race = readmatrix("YasMarina.csv");
path_x = race(4:end,2);
path_y = race(4:end,3);

%% =========================================================
%                VEHICLE PARAMETERS (Dynamic)
% =========================================================
L_f = 0.9;      L_r = 0.7;      L = L_f + L_r;
Mass = 1200;    Iz = 2000;
Cf = 120000;    Cr = 120000;
dt = 0.02;      mu = 1.2;
v_max = 50;     v_switch = 5.0; 

%% =========================================================
%                STANLEY & PID GAINS
% =========================================================
k_stanley = 1.5;    v_eps = 1.0;
Kp_v = 1.5;         Ki_v = 0.2;

%% =========================================================
%        ARC-LENGTH REPARAMETERIZATION & SPEED PROFILE
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

% Setup UI
fig = figure('Name','Vehicle Dynamics Telemetry - Yas Marina','Color','k','Units','normalized','Position',[0.05 0.05 0.9 0.85]);

% Main Map
ax1 = subplot(4,4,[1 2 5 6 9 10 13 14]); 
plot(ax1, path_x, path_y, 'w--', 'LineWidth', 1); hold on;
h_trail = plot(ax1, X, Y, 'c-', 'LineWidth', 1.5);
h_car   = plot(ax1, X, Y, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
title(ax1, 'Stanley Algorithm', 'Color', 'w');
axis(ax1, 'equal'); grid(ax1, 'on'); set(ax1, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');

% Speed Plot
ax2 = subplot(4,4,[3 4]); h_vel = animatedline(ax2, 'Color', 'g', 'LineWidth', 2);
h_vref = animatedline(ax2, 'Color', [0.5 0.5 0.5], 'LineStyle', '--');
ylabel(ax2, 'Speed [m/s]', 'Color', 'w'); grid on; title(ax2, 'Longitudinal Velocity', 'Color', 'w');
set(ax2, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');

% Steering Plot
ax3 = subplot(4,4,[7 8]); h_delta = animatedline(ax3, 'Color', 'm', 'LineWidth', 1.5);
ylabel(ax3, 'Delta [deg]', 'Color', 'w'); grid on; title(ax3, 'Steering Command', 'Color', 'w');
set(ax3, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');

% Lateral Velocity Plot (Slip Indicator)
ax4 = subplot(4,4,[11 12]); h_vy = animatedline(ax4, 'Color', 'y', 'LineWidth', 1.5);
ylabel(ax4, 'Vy [m/s]', 'Color', 'w'); grid on; title(ax4, 'Lateral Slip (Dynamic Effect)', 'Color', 'w');
set(ax4, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');

% Model Indicator Text
h_text = annotation('textbox', [0.75, 0.05, 0.2, 0.05], 'String', 'Model: INITIALIZING', ...
    'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold', 'EdgeColor', 'y', 'BackgroundColor', 'k');

% Lap Time Display (NEW)
h_timer = annotation('textbox', [0.05, 0.05, 0.2, 0.05], 'String', 'Lap Time: 0.00s', ...
    'Color', 'g', 'FontSize', 14, 'FontWeight', 'bold', 'EdgeColor', 'g', 'BackgroundColor', 'k');

%% =========================================================
%                MAIN SIMULATION LOOP
% =========================================================
traj_x = []; traj_y = [];
fprintf('Running Hybrid Model Simulation...\n');

for k = 1:150000
    % 1. Sensing & Errors
    xf = X + L_f * cos(PSI); yf = Y + L_f * sin(PSI);
    idx_end = min(idx_closest+100, Npath);
    [~,i_m] = min((path_x(idx_closest:idx_end)-xf).^2 + (path_y(idx_closest:idx_end)-yf).^2);
    idx_closest = idx_closest + i_m - 1;
    idx_closest = min(idx_closest, Npath-1);
    
    path_psi = atan2(path_y(idx_closest+1)-path_y(idx_closest), path_x(idx_closest+1)-path_x(idx_closest));
    e_ct = (path_y(idx_closest)-yf)*cos(path_psi) - (path_x(idx_closest)-xf)*sin(path_psi);
    theta_e = atan2(sin(path_psi - PSI), cos(path_psi - PSI));

    % 2. Control
    delta = theta_e + atan2(k_stanley * e_ct, vx + v_eps);
    delta = max(min(delta, deg2rad(25)), -deg2rad(25));
    v_ref = v_race(idx_closest);
    v_err = v_ref - vx; vel_int = vel_int + v_err*dt;
    acc = Kp_v*v_err + Ki_v*vel_int;

    % 3. Physics Switching
    if vx < v_switch
        beta = atan((L_r / L) * tan(delta));
        dX = vx * cos(PSI + beta); dY = vx * sin(PSI + beta); dPSI = (vx / L_r) * sin(beta);
        dvx = acc; dvy = 0; dr = 0;
        model_str = 'Model: KINEMATIC (Low Speed)';
    else
        alpha_f = delta - atan2((vy + L_f*r), vx);
        alpha_r = -atan2((vy - L_r*r), vx);
        Fyf = Cf * alpha_f; Fyr = Cr * alpha_r;
        dX = vx*cos(PSI) - vy*sin(PSI); dY = vx*sin(PSI) + vy*cos(PSI);
        dPSI = r; dvx = acc;
        dvy = (Fyf + Fyr) / Mass - vx*r;
        dr = (L_f*Fyf - L_r*Fyr) / Iz;
        model_str = 'Model: DYNAMIC (High Speed)';
    end

    % 4. Integration
    X = X + dX*dt; Y = Y + dY*dt; PSI = PSI + dPSI*dt;
    vx = max(vx + dvx*dt, 0.1); vy = vy + dvy*dt; r = r + dr*dt;

    % 5. Live Plotting & Timer Update
    traj_x(end+1) = X; traj_y(end+1) = Y;
    curr_t = k * dt; % Total simulation time

    if mod(k,8) == 0 % Update display frequency
        addpoints(h_vel, curr_t, vx); addpoints(h_vref, curr_t, v_ref);
        addpoints(h_delta, curr_t, rad2deg(delta));
        addpoints(h_vy, curr_t, vy);
        
        set(h_car, 'XData', X, 'YData', Y);
        set(h_trail, 'XData', traj_x, 'YData', traj_y);
        set(h_text, 'String', model_str);
        set(h_timer, 'String', sprintf('Lap Time: %.2fs', curr_t));
        
        % Dynamic axis scaling
        xlim(ax2, [max(0, curr_t-5) curr_t+1]);
        xlim(ax3, [max(0, curr_t-5) curr_t+1]);
        xlim(ax4, [max(0, curr_t-5) curr_t+1]);
        
        drawnow limitrate;
    end
    
    % Exit Condition: Lap Completion
    if idx_closest >= Npath-5
        fprintf('Lap Completed! Total Time: %.2fs\n', curr_t);
        set(h_timer, 'String', sprintf('FINISH: %.2fs', curr_t), 'EdgeColor', 'y');
        break;
    end
end