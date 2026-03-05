function P = quad_params()
% QUAD_PARAMS
% Loads physical parameters + computes LQR gains (once).

%% ============================================================
% Physical parameters
% ============================================================
P.m = 1.0;               % mass (kg)
P.g = 9.81;              % gravity

Ix = 0.02; Iy = 0.02; Iz = 0.04;
P.I = diag([Ix Iy Iz]);  % inertia

P.L  = 0.25;             % arm length
P.kf = 1e-5;             % thrust coefficient
P.km = 2e-6;             % drag coefficient

%% Force/torque limits
P.FT_min = 0;
P.FT_max = 30;           % max total thrust (N)
P.F_min  = 0;
P.F_max  = 7.5;          % per-motor

P.tau_max = 1.5;         % max torque along axes (Nm)

%% ============================================================
% LQR Weighting Matrices (same as paper)
% ============================================================
P.Q_pos = diag([200 200 1000  1 1 1]);   % for [p,v]
P.R_pos = diag([10 10 1]);

P.Q_att = diag([100 100 1000 10 10 10]); % for [angles,rates]
P.R_att = diag([100 100 100]);

%% ============================================================
% Compute LQR Gains Once
% ============================================================

% ----- Translational subsystem -----
A_pos = [zeros(3), eye(3);
         zeros(3), zeros(3)];

B_pos = [zeros(3,3);
         eye(3)];

P.K_pos = lqr(A_pos, B_pos, P.Q_pos, P.R_pos);


% ----- Attitude subsystem -----
A_att = [zeros(3), eye(3);
         zeros(3,6)];

B_att = [zeros(3,3);
         inv(P.I)];

P.K_att = lqr(A_att, B_att, P.Q_att, P.R_att);

%% Controller dt
P.dt_ctrl = 0.01;

end
