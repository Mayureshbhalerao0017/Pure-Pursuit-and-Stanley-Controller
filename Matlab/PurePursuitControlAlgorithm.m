clc; clear; close all;

%% =========================================================
%                LOAD RACING LINE CSV
%% =========================================================
race = readmatrix("traj_race_cl.csv");

% Correct columns from your file
% col 1 = arc length s
% col 2 = x [m]
% col 3 = y [m]
path_x = race(4:end,2);
path_y = race(4:end,3);
Npath  = length(path_x);

%% =========================================================
%                VEHICLE PARAMETERS
%% =========================================================
L  = 1.6;        % wheelbase [m]
dt = 0.01;       % timestep [s]
T  = 130;        % max simulation time
N  = round(T/dt);

mu    = 1.2;
v_max = 30;

%% =========================================================
%                PURE PURSUIT PARAMETERS
%% =========================================================
Ld_min = 1.2;
Ld_max = 4.0;
k_Ld   = 0.3;

%% =========================================================
%                SPEED PID GAINS
%% =========================================================
Kp_v = 1.5;
Ki_v = 0.2;
Kd_v = 0.05;

%% =========================================================
%                INITIAL STATE
%% =========================================================
x = path_x(1);
y = path_y(1);

dx0 = path_x(2) - path_x(1);
dy0 = path_y(2) - path_y(1);
psi = atan2(dy0, dx0);

v = 2.0;

vel_int  = 0;
vel_prev = 0;

idx_closest = 1;
lap_count = 0;

%% =========================================================
%                LOGGING
%% =========================================================
Xlog = zeros(N,1);
Ylog = zeros(N,1);
Vlog = zeros(N,1);

%% =========================================================
%                MAIN SIMULATION LOOP
%% =========================================================
for k = 1:N

    %% ---------- FIND CLOSEST POINT ----------
    search_window = 30;
    idx_end = min(idx_closest + search_window, Npath);

    dx = path_x(idx_closest:idx_end) - x;
    dy = path_y(idx_closest:idx_end) - y;

    [~, i_min] = min(dx.^2 + dy.^2);
    idx_closest = idx_closest + i_min - 1;

    % Safe circular indexing
    idx_closest = mod(idx_closest-1, Npath) + 1;

    % Lap detection
    if idx_closest == 1 && k > 200
        lap_count = lap_count + 1;
        disp("Lap completed");
        break;   % stop after one lap (for clean plot)
    end

    %% ---------- LOOKAHEAD DISTANCE ----------
    Ld = min(max(Ld_min + k_Ld*v, Ld_min), Ld_max);

    %% ---------- FIND LOOKAHEAD POINT ----------
    dist = 0;
    idx_L = idx_closest;

    while dist < Ld
        idx_next = idx_L + 1;
        if idx_next > Npath
            idx_next = 1;
        end

        dist = dist + hypot(path_x(idx_next)-path_x(idx_L), ...
                             path_y(idx_next)-path_y(idx_L));
        idx_L = idx_next;
    end

    xL = path_x(idx_L);
    yL = path_y(idx_L);

    %% =========================================================
    %                  PURE PURSUIT STEERING
    %% =========================================================
    dx =  cos(psi)*(xL-x) + sin(psi)*(yL-y);
    dy = -sin(psi)*(xL-x) + cos(psi)*(yL-y);

    alpha = atan2(dy, dx);
    delta = atan2(2*L*sin(alpha), Ld);
    delta = max(min(delta, deg2rad(25)), -deg2rad(25));

    %% =========================================================
    %                  SPEED PLANNING
    %% =========================================================
    kappa = 2*sin(alpha)/Ld;
    ay_max = mu * 9.81;

    v_ref = sqrt(ay_max / max(abs(kappa), 0.05));
    v_ref = min(max(v_ref, 2.0), v_max);

    %% =========================================================
    %                  SPEED PID
    %% =========================================================
    vel_err = v_ref - v;
    vel_int = vel_int + vel_err*dt;
    vel_der = (vel_err - vel_prev)/dt;

    acc_cmd = Kp_v*vel_err + Ki_v*vel_int + Kd_v*vel_der;
    vel_prev = vel_err;

    acc_cmd = max(min(acc_cmd, 2.0), -3.0);

    %% =========================================================
    %                  VEHICLE UPDATE
    %% =========================================================
    v   = max(v + acc_cmd*dt, 1.0);
    psi = psi + (v/L)*tan(delta)*dt;
    psi = atan2(sin(psi), cos(psi));

    x = x + v*cos(psi)*dt;
    y = y + v*sin(psi)*dt;

    %% ---------- LOG ----------
    Xlog(k) = x;
    Ylog(k) = y;
    Vlog(k) = v;
end

%% =========================================================
%                     CLEAN LOGS
%% =========================================================
valid = ~(Xlog == 0 & Ylog == 0);
Xlog = Xlog(valid);
Ylog = Ylog(valid);
Vlog = Vlog(valid);

%% =========================================================
%                     PLOTS
%% =========================================================
figure;
plot(path_x, path_y, 'w--','LineWidth',1.5); hold on;
plot(Xlog, Ylog, 'b--','LineWidth',2.5);
axis equal; grid on;
legend("Racing Line","Vehicle Trajectory");
title("Pure Pursuit Path Tracking for Modena");

figure;
plot((0:length(Vlog)-1)*dt, Vlog,'LineWidth',2);
xlabel("Time [s]");
ylabel("Speed [m/s]");
grid on;
title("Vehicle Speed Profile for Modena");
