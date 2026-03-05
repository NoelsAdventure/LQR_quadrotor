function [u, state] = controller_LQR(t, x, ref, P, state)
% CONTROLLER_LQR
% Cascaded LQR controller:
% 1. Outer-loop (position) → desired accelerations
% 2. Convert to desired roll/pitch
% 3. Inner-loop (attitude) → torques
%
% Output wrench u = [ft; τφ; τθ; τψ]

%% ============================================================
% Extract states
% ============================================================
x_pos = x(1:6);             % [p; v]
xr_pos = [ref.p_ref; ref.v_ref];

x_att = x(7:12);            % [angles; body rates]

psi = x(9);

%% ============================================================
% OUTER LOOP — Translational LQR
% ============================================================
u_pos = -P.K_pos * (x_pos - xr_pos);   % desired accelerations ax,ay,az

% desired collective thrust
ft = P.m * (P.g + u_pos(3));

% thrust saturation
ft = min(max(ft, P.FT_min), P.FT_max);

% desired roll & pitch (from Eq. (15) in the paper)
phi_des   = (1/P.g) * (u_pos(1)*sin(psi) - u_pos(2)*cos(psi));
theta_des = (1/P.g) * (u_pos(1)*cos(psi) + u_pos(2)*sin(psi));

%% ============================================================
% INNER LOOP — Attitude LQR
% ============================================================
xr_att = [phi_des; theta_des; ref.yaw_ref; 0;0;0];

tau = -P.K_att * (x_att - xr_att);   % [τφ; τθ; τψ]

% torque saturation
tau = max(min(tau, P.tau_max), -P.tau_max);

%% ============================================================
% OUTPUT
% ============================================================
u = [ft; tau];
state = state;   % not used for LQR (placeholder)

end
