function [delta_cmd, acc_cmd] = vehicleController(state, path)
% state.pose = [X; Y; PSI]      (from EKF-SLAM)
% state.vel  = [vx; vy; r]      (from Vehicle EKF)
% path.x, path.y, path.v_ref

%% ================= PERSISTENT PARAMETERS =================
persistent p idx_closest vel_int
if isempty(p)
    % Vehicle geometry
    p.L_f = 0.783;
    p.L_r = 0.752;
    p.dt  = 0.01;

    % Lateral control
    p.k_stanley = 1.5;
    p.k_pure    = 0.8;
    p.L_min     = 3.0;
    p.v_eps     = 0.5;
    p.max_delta = deg2rad(25);

    % Longitudinal control
    p.Kp_v = 1.5;
    p.Ki_v = 0.2;
    p.max_acc   = 5.0;
    p.max_decel = -8.0;

    idx_closest = 1;
    vel_int = 0;
end

%% ================= UNPACK STATES =================
X   = state.pose(1);
Y   = state.pose(2);
PSI = state.pose(3);

vx  = max(state.vel(1), 0.5);   % 🚨 critical fix
vy  = state.vel(2);
r   = state.vel(3);

%% ================= FRONT AXLE REFERENCE =================
xf = X + p.L_f * cos(PSI);
yf = Y + p.L_f * sin(PSI);

L_ld = p.k_pure * vx + p.L_min;

%% ================= PATH SEARCH =================
Npath = length(path.x);
search_end = min(idx_closest + 100, Npath);

[~, i_local] = min((path.x(idx_closest:search_end) - xf).^2 + ...
                   (path.y(idx_closest:search_end) - yf).^2);

idx_closest = idx_closest + i_local - 1;

%% ================= LOOKAHEAD TARGET =================
idx_ld = idx_closest;
while idx_ld < Npath && hypot(path.x(idx_ld)-xf, path.y(idx_ld)-yf) < L_ld
    idx_ld = idx_ld + 1;
end

tx = path.x(idx_ld);
ty = path.y(idx_ld);

%% ================= PATH HEADING =================
if idx_ld < Npath
    path_psi = atan2(path.y(idx_ld+1) - path.y(idx_ld), ...
                     path.x(idx_ld+1) - path.x(idx_ld));
else
    path_psi = atan2(path.y(idx_ld) - path.y(idx_ld-1), ...
                     path.x(idx_ld) - path.x(idx_ld-1));
end

%% ================= LATERAL CONTROL =================
e_ct =  (ty - yf)*cos(path_psi) - (tx - xf)*sin(path_psi);
theta_e = wrapToPi(path_psi - PSI);

delta_cmd = theta_e + atan2(p.k_stanley * e_ct, vx + p.v_eps);
delta_cmd = max(min(delta_cmd, p.max_delta), -p.max_delta);

%% ================= LONGITUDINAL CONTROL =================
v_ref = path.v_ref(idx_closest);
v_err = v_ref - vx;

vel_int = vel_int + v_err * p.dt;
acc_cmd = p.Kp_v * v_err + p.Ki_v * vel_int;

acc_cmd = max(min(acc_cmd, p.max_acc), p.max_decel);

end
