function [traj_x, traj_y, v_history] = run_stanley_simulation()
    %% =========================================================
    %                LOAD TRACK & PRE-PROCESSING
    %% =========================================================
    % Load provided skidpad trajectory 
    try
        race = readmatrix("traj_race_cl.csv");
    catch
        error("Could not find 'skidpad_trajectory.csv'. Ensure the file is in the current directory.");
    end
    
    path_x_raw = race(2:end, 2);
    path_y_raw = race(2:end, 3);

    % Arc-Length Reparameterization for smooth interpolation [cite: 6, 7]
    dx = diff(path_x_raw);
    dy = diff(path_y_raw);
    ds = hypot(dx, dy);
    s_raw = [0; cumsum(ds)];
    [s_raw, unique_idx] = unique(s_raw); % Ensure strictly increasing for interpolation
    x_v = path_x_raw(unique_idx);
    y_v = path_y_raw(unique_idx);
    
    % Create a uniform path with 10cm spacing for high-fidelity control [cite: 8]
    s_uniform = 0:0.1:s_raw(end);
    path_x = interp1(s_raw, x_v, s_uniform, 'pchip')';
    path_y = interp1(s_raw, y_v, s_uniform, 'pchip')';
    Npath  = length(path_x);

    %% =========================================================
    %                VEHICLE & CONTROLLER PARAMETERS
    %% =========================================================
    L_f = 1.535 * 0.51; % Front length [cite: 2]
    L_r = 1.535 * 0.49; % Rear length [cite: 3]
    L   = L_f + L_r;    % Total wheelbase [cite: 3]
    dt  = 0.02;         % Simulation time step [cite: 3]
    mu  = 0.5;          % Friction coefficient [cite: 4]
    v_max = 20;         % Speed limit [cite: 4]
    
    k_stanley = 0.5;    % Stanley gain [cite: 4]
    v_eps     =3;    % Softening gain [cite: 5]
    
    Kp_v = 1.5; Ki_v = 0.5; Kd_v = 0.02; % Speed PID [cite: 5]

    %% =========================================================
    %        OFFLINE RACING SPEED PROFILE [cite: 9, 10]
    %% =========================================================
    kappa_track = zeros(Npath, 1);
    for i = 2:Npath-1
        psi1 = atan2(path_y(i)-path_y(i-1), path_x(i)-path_x(i-1));
        psi2 = atan2(path_y(i+1)-path_y(i), path_x(i+1)-path_x(i));
        dpsi = atan2(sin(psi2-psi1), cos(psi2-psi1)); % Normalize dpsi
        ds_seg = hypot(path_x(i+1)-path_x(i), path_y(i+1)-path_y(i));
        kappa_track(i) = dpsi / max(ds_seg, 1e-3);
    end
    
    ay_max = mu * 9.81;
    v_race = min(sqrt(ay_max ./ max(abs(kappa_track), 1e-3)), v_max);
    
    % Smooth velocity profile with braking/acceleration limits
    a_brake = 6.0; a_drive = 4.0;
    for i = Npath-1:-1:1
        ds_seg = hypot(path_x(i+1)-path_x(i), path_y(i+1)-path_y(i));
        v_race(i) = min(v_race(i), sqrt(v_race(i+1)^2 + 2*a_brake*ds_seg));
    end
    for i = 2:Npath
        ds_seg = hypot(path_x(i)-path_x(i-1), path_y(i)-path_y(i-1));
        v_race(i) = min(v_race(i), sqrt(v_race(i-1)^2 + 2*a_drive*ds_seg));
    end

    %% =========================================================
    %                INITIAL STATE & VIZ SETUP [cite: 13-22]
    %% =========================================================
    x_car = path_x(1); y_car = path_y(1);
    psi = atan2(path_y(2)-path_y(1), path_x(2)-path_x(1));
    v = 5.0; idx_closest = 1; vel_int = 0; vel_prev = 0;
    
    N_max = round(500/dt);
    traj_x = zeros(N_max, 1); traj_y = zeros(N_max, 1); v_history = zeros(N_max, 1);
    
    fig = figure('Name','Real-Time Stanley Racing','Color','k','Units','normalized','Position',[0.1 0.1 0.8 0.8]);
    ax1 = subplot(3,2,[1 2 3 4]);
    plot(ax1, path_x, path_y, 'w--', 'LineWidth', 1.2); hold on;
    h_trail = plot(ax1, x_car, y_car, 'c-', 'LineWidth', 1.5);
    h_car   = plot(ax1, x_car, y_car, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    h_orient = quiver(ax1, x_car, y_car, cos(psi), sin(psi), 2, 'y', 'LineWidth', 2);
    axis(ax1, 'equal'); grid(ax1, 'on'); set(ax1, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');
    
    ax2 = subplot(3,2,5); h_vel = animatedline(ax2, 'Color', 'g', 'LineWidth', 2);
    h_ref = animatedline(ax2, 'Color', 'w', 'LineStyle', '--'); ylabel('m/s'); grid on;
    set(ax2, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');
    
    ax3 = subplot(3,2,6); h_steer = animatedline(ax3, 'Color', 'm'); ylabel('Deg'); grid on;
    set(ax3, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');

    h_timer = annotation('textbox', [0.15, 0.82, 0.2, 0.05], 'String', 'Lap Time: 0.00s', ...
        'Color', 'g', 'FontSize', 14, 'FontWeight', 'bold', 'EdgeColor', 'g', 'BackgroundColor', 'k');

    %% =========================================================
    %                MAIN REAL-TIME LOOP [cite: 23-24]
    %% =========================================================
    k = 1;
    start_sim_time = tic; 

    while k <= N_max
        % Real-Time Sync 
        while toc(start_sim_time) < (k * dt), end
        
        % 1. Stanley Front Axle Reference [cite: 25]
        x_f = x_car + L_f * cos(psi);
        y_f = y_car + L_f * sin(psi);

        % 2. Robust Nearest-Point Search [cite: 27]
        search_window = 100;
        idx_end = min(idx_closest + search_window, Npath-1);
        dxp = path_x(idx_closest:idx_end) - x_f;
        dyp = path_y(idx_closest:idx_end) - y_f;
        [~, i_min] = min(dxp.^2 + dyp.^2);
        idx_closest = min(max(idx_closest + i_min - 1, 1), Npath-1);

        % 3. Cross-Track Error (e_ct) 
        px = path_x(idx_closest); py = path_y(idx_closest);
        qx = path_x(idx_closest+1); qy = path_y(idx_closest+1);
        sx = qx-px; sy = qy-py; seg_len = hypot(sx, sy);
        
        % Vector from path point to car front axle
        dx_car = x_f - px; dy_car = y_f - py;
        % Normal vector to path (pointing left)
        nx = -sy/seg_len; ny = sx/seg_len;
        e_ct = dx_car * nx + dy_car * ny;

        % 4. Heading Error (theta_e) - Normalized 
        psi_path = atan2(sy, sx);
        theta_e = atan2(sin(psi_path - psi), cos(psi_path - psi));

        % 5. Steering Control Law 
        delta = theta_e + atan2(k_stanley * e_ct, v + v_eps);
        delta = max(min(delta, deg2rad(25)), -deg2rad(25)); % Limit to 25 deg [cite: 37]

        % 6. Speed Control (PID + Friction Circle) [cite: 37, 38]
        v_ref = v_race(idx_closest);
        vel_err = v_ref - v;
        vel_int = vel_int + vel_err * dt;
        acc_cmd = Kp_v * vel_err + Ki_v * vel_int + Kd_v * ((vel_err - vel_prev) / dt);
        vel_prev = vel_err;
        
        % Limit acceleration based on lateral grip
        ay = v^2 * abs(kappa_track(idx_closest));
        ratio = ay / (mu * 9.81);
        ax_max = mu * 9.81 * sqrt(max(0, 1 - ratio^2));
        acc_cmd = max(min(acc_cmd, ax_max), -ax_max);

        % 7. Physics Integration (Kinematic Bicycle) [cite: 39, 40]
        v   = max(v + acc_cmd * dt, 0.1);
        psi = psi + (v / L) * tan(delta) * dt;
        x_car = x_car + v * cos(psi) * dt;
        y_car = y_car + v * sin(psi) * dt;
        
        traj_x(k) = x_car; traj_y(k) = y_car; v_history(k) = v;

        % 8. Visualization Update [cite: 49-51]
        if mod(k, 10) == 0 && ishandle(fig)
            set(h_car, 'XData', x_car, 'YData', y_car);
            set(h_trail, 'XData', traj_x(1:k), 'YData', traj_y(1:k));
            set(h_orient, 'XData', x_car, 'YData', y_car, 'UData', 3*cos(psi), 'VData', 3*sin(psi));
            set(h_timer, 'String', sprintf('Lap Time: %.2fs', toc(start_sim_time)));
            addpoints(h_vel, k*dt, v); addpoints(h_ref, k*dt, v_ref);
            addpoints(h_steer, k*dt, rad2deg(delta));
            drawnow limitrate;
        end

        if idx_closest >= Npath-5, break; end
        k = k + 1;
    end
    
    % Finalization
    total_time = toc(start_sim_time);
    set(h_timer, 'String', sprintf('Final Time: %.2fs', total_time), 'Color', 'y');
    traj_x = traj_x(1:k); traj_y = traj_y(1:k); v_history = v_history(1:k);
    fprintf('Lap Completed in %.2f seconds.\n', total_time);
end