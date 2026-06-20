clc; clear; close all;

%% =========================================================
%                LOAD RACING LINE CSV
%% =========================================================
race = readmatrix("traj_race_cl.csv");

path_x = race(4:end,2);
path_y = race(4:end,3);
Npath  = length(path_x);

%% =========================================================
%                VEHICLE PARAMETERS
%% =========================================================
L  = 1.6;        % wheelbase [m]
dt = 0.01;
T  = 130;
N  = round(T/dt);

mu    = 1.2;
v_max = 30;

%% =========================================================
%                STANLEY PARAMETERS
%% =========================================================
k_stanley = 1.0;     % cross-track gain (stable)
v_eps     = 1.0;     % low-speed regularization

%% =========================================================
%                SPEED PID GAINS
%% =========================================================
Kp_v = 1.5;
Ki_v = 0.2;
Kd_v = 0.05;

%% =========================================================
%                INITIAL STATE (ON PATH)
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

    %% ---------- FRONT AXLE POSITION ----------
    x_f = x + L*cos(psi);
    y_f = y + L*sin(psi);

    %% ---------- FIND CLOSEST PATH SEGMENT ----------
    search_window = 40;
    idx_end = min(idx_closest + search_window, Npath-1);

    dxp = path_x(idx_closest:idx_end) - x_f;
    dyp = path_y(idx_closest:idx_end) - y_f;

    [~, i_min] = min(dxp.^2 + dyp.^2);
    idx_closest = idx_closest + i_min - 1;
    idx_closest = min(idx_closest, Npath-1);

    %% =========================================================
    %               STANLEY STEERING (WITH FEEDFORWARD)
    %% =========================================================

    % ---- Path segment ----
    px = path_x(idx_closest);
    py = path_y(idx_closest);
    qx = path_x(idx_closest+1);
    qy = path_y(idx_closest+1);

    sx = qx - px;
    sy = qy - py;
    s2 = sx^2 + sy^2;

    % ---- Projection onto segment ----
    t = ((x_f-px)*sx + (y_f-py)*sy) / s2;
    t = max(0,min(1,t));

    x_proj = px + t*sx;
    y_proj = py + t*sy;

    % ---- Path heading ----
    psi_path = atan2(sy, sx);

    % ---- Heading error ----
    theta_e = atan2(sin(psi_path-psi), cos(psi_path-psi));

    % ---- Signed cross-track error ----
    nx = -sin(psi_path);
    ny =  cos(psi_path);
    e_ct = (x_f-x_proj)*nx + (y_f-y_proj)*ny;

    %% ---------- CURVATURE FEEDFORWARD ----------
    kappa = 0;
    if idx_closest > 1 && idx_closest < Npath-1
        x1 = path_x(idx_closest-1); y1 = path_y(idx_closest-1);
        x2 = path_x(idx_closest);   y2 = path_y(idx_closest);
        x3 = path_x(idx_closest+1); y3 = path_y(idx_closest+1);

        a = hypot(x2-x1, y2-y1);
        b = hypot(x3-x2, y3-y2);
        c = hypot(x3-x1, y3-y1);

        kappa = 2*abs((x2-x1)*(y3-y1)-(y2-y1)*(x3-x1)) ...
                / max(a*b*c,1e-3);
    end

    delta_ff = atan(L * kappa);   % feedforward steering

    %% ---------- STANLEY CONTROL LAW ----------
    delta = theta_e ...
          + atan2(k_stanley * e_ct, v + v_eps) ...
          + delta_ff;

    % Steering limits
    delta = max(min(delta, deg2rad(25)), -deg2rad(25));

    %% =========================================================
    %                  SPEED PLANNING
    %% =========================================================
    ay_max = mu * 9.81;
    v_ref = sqrt(ay_max / max(kappa,0.05));
    v_ref = min(max(v_ref,2.0), v_max);

    %% =========================================================
    %                  SPEED PID
    %% =========================================================
    vel_err = v_ref - v;
    vel_int = vel_int + vel_err*dt;
    vel_der = (vel_err - vel_prev)/dt;

    acc_cmd = Kp_v*vel_err + Ki_v*vel_int + Kd_v*vel_der;
    vel_prev = vel_err;

    acc_cmd = max(min(acc_cmd,2.0), -3.0);

    %% =========================================================
    %                  VEHICLE UPDATE
    %% =========================================================
    v   = max(v + acc_cmd*dt,1.0);
    psi = psi + (v/L)*tan(delta)*dt;
    psi = atan2(sin(psi),cos(psi));

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
valid = ~(Xlog==0 & Ylog==0);
Xlog = Xlog(valid);
Ylog = Ylog(valid);
Vlog = Vlog(valid);

%% =========================================================
%                     PLOTS
%% =========================================================
figure;
plot(path_x,path_y,'w--','LineWidth',1.5); hold on;
plot(Xlog,Ylog,'b','LineWidth',2.5);
axis equal; grid on;
legend("Racing Line","Vehicle Trajectory");
title("Stanley Path Tracking with Feedforward (FINAL)");

figure;
plot((0:length(Vlog)-1)*dt,Vlog,'LineWidth',2);
xlabel("Time [s]");
ylabel("Speed [m/s]");
grid on;
title("Speed Profile – Stanley + Feedforward");
