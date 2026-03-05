function sim_quadcopter()
% SIM_QUADCOPTER
% -------------------------------------------------------------
% Full nonlinear simulation of quadrotor tracking using
% cascaded LQR:
%
% Outer loop → translational LQR → desired accelerations
% Convert → roll/pitch desired
% Inner loop → rotational LQR → body torques
% Mixer → rotor forces
%
% Implements exactly the pipeline shown in Fig. 2 of paper.
% -------------------------------------------------------------

clear; close all; clc;

%% ============================================================
% Load reference trajectory
% ============================================================
trajectory = 'circle';   % circle | helix | lemniscate
generate_trajectory(trajectory);

S = load('Trajectory_of_Drone.mat');
traj = S.traj;

T = traj.t(:)';
N = numel(T);

%% ============================================================
% Parameters & Gains
% ============================================================
P = quad_params();   % Now includes LQR gains

%% Initial state
X = zeros(12,N);
X(1:3,1) = traj.p_ref(1,:)' + [0;2;0];
X(4,1)   = 0.1;
X(9,1)   = traj.yaw_ref(1);

F_log = zeros(4,N);

%% Mixer matrix (Eq. (1))
L = P.L;
km_kf = P.km / P.kf;

D = [ 1      1      1      1   ;
      0     -L      0      L   ;
     -L      0      L      0   ;
     -km_kf  km_kf -km_kf  km_kf ];

%% ============================================================
% Simulation loop
% ============================================================
state = struct();

for k = 1:N-1
    dt = T(k+1)-T(k);

    % Reference at step
    ref.p_ref   = traj.p_ref(k,:)';
    ref.v_ref   = traj.v_ref(k,:)';
    ref.yaw_ref = traj.yaw_ref(k);

    % LQR controller → desired wrench
    [u_des, state] = controller_LQR(T(k), X(:,k), ref, P, state);

    % Allocate thrusts
    F = D \ u_des;

    % Clip per-rotor forces
    F = max(min(F, P.F_max), P.F_min);

    F_log(:,k) = F;

    % RK4 propagation
    k1 = quad_dynamics(T(k),        X(:,k),           F, P);
    k2 = quad_dynamics(T(k)+dt/2.0, X(:,k)+dt/2.0*k1, F, P);
    k3 = quad_dynamics(T(k)+dt/2.0, X(:,k)+dt/2.0*k2, F, P);
    k4 = quad_dynamics(T(k)+dt,     X(:,k)+dt*k3,     F, P);

    X(:,k+1) = X(:,k) + dt/6*(k1 + 2*k2 + 2*k3 + k4);

    X(9,1:k+1) = unwrap(X(9,1:k+1));
end

%% ============================================================
% Last control input
% ============================================================
u_last = controller_LQR(T(end), X(:,end), ref, P, state);
F_log(:,end) = D \ u_last;

%% ============================================================
% Plots
% ============================================================
figure('Color','w');
tiledlayout(2,1);

% Position error
nexttile;
pos_err = traj.p_ref' - X(1:3,:);
plot(T, pos_err','LineWidth',1.6); grid on;
ylabel('Position Error (m)');
legend('e_x','e_y','e_z');

% Rotor forces
nexttile;
plot(T, F_log','LineWidth',1.6); grid on;
ylabel('Rotor Forces (N)');
xlabel('Time (s)');
ylim([0 7.5]);
legend('f_1','f_2','f_3','f_4');

saveas(gcf, fullfile('results', ['TrackingAndControl_', trajectory, '.png']));

%% ============================================================
% Metrics (as in Table I)
% ============================================================
pos_norm = vecnorm(pos_err,2,1);

rmse_x = sqrt(mean(pos_err(1,:).^2));
rmse_y = sqrt(mean(pos_err(2,:).^2));
rmse_z = sqrt(mean(pos_err(3,:).^2));
rmse_pos = sqrt(mean(pos_norm.^2));

max_err = max(pos_norm);
ATE = mean(pos_norm);

fprintf('\n========== Evaluation Metrics ==========\n');
fprintf('RMSE_x     = %.4f m\n', rmse_x);
fprintf('RMSE_y     = %.4f m\n', rmse_y);
fprintf('RMSE_z     = %.4f m\n', rmse_z);
fprintf('RMSE_pos   = %.4f m\n', rmse_pos);
fprintf('Max Error  = %.4f m\n', max_err);
fprintf('ATE        = %.4f m\n', ATE);
fprintf('========================================\n');


%% ============================================================
% Animatioin
%% ============================================================
animate_quadcopter(T, X, traj, trajectory, P);

end
