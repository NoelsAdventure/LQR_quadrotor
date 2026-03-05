function generate_trajectory(shape)
%GENERATE_TRAJECTORY Create a 3D reference and save Trajectory_of_Drone.mat
% Usage:
%   generate_trajectory('circle'|'lemniscate'|'helix'|'custom')

if nargin<1, shape = 'lemniscate'; end

Tf = 40;                 % total time (s)
dt = 0.02;               % sample time (s)
t = (0:dt:Tf)';          % [Nx1]
w = 2*pi/Tf*2;           % angular speed

switch lower(shape)
    case 'circle'
        R = 5;
        x = R*cos(w*t);
        y = R*sin(w*t);
        z = 2 + 0*t;
    case 'lemniscate' % figure-8
        a = 5;
        x = a*sin(w*t);
        y = a*sin(w*t).*cos(w*t);
        z = 2 + 0.5*sin(0.5*w*t);
    case 'helix'
        R = 4;
        x = R*cos(w*t);
        y = R*sin(w*t);
        z = linspace(0.5, 5.5, numel(t))';
    case 'custom'
        W = [ 0  0  1;
              5  0  2;
              5  5  3;
              0  5  2;
             -4  2  1.5];
        nseg = size(W,1)-1;
        t_wp = linspace(0, Tf, nseg+1);
        x = spline(t_wp, [W(1,1) W(:,1)' W(end,1)], t);
        y = spline(t_wp, [W(1,2) W(:,2)' W(end,2)], t);
        z = spline(t_wp, [W(1,3) W(:,3)' W(end,3)], t);
    case 'step'
        Tf = 5;                 % total time (s)
        dt = 0.02;               % sample time (s)
        t = (0:dt:Tf)';          % [Nx1]
        w = 2*pi/Tf*2;           % angular speed

        x = 10*ones(size(t));
        y = 0*ones(size(t));
        z = 1*ones(size(t));  

        x(1) = 0;
        y(1) = 0;
        z(1) = 0;
    otherwise
        error('Unknown shape "%s".', shape);
end

% Yaw faces the path tangent
yaw = atan2(diff([y(1); y]), diff([x(1); x]));   % length N

% Finite-difference reference velocity
xd = [0; diff(x)/dt];
yd = [0; diff(y)/dt];
zd = [0; diff(z)/dt];

% if shape == 'step'
%     xd(2) = 0;
%     yd(2) = 0;
%     zd(2) = 0;
% end

traj.t       = t;
traj.p_ref   = [x y z];
traj.v_ref   = [xd yd zd];
traj.yaw_ref = unwrap(yaw);

save('Trajectory_of_Drone.mat', 'traj');
fprintf('Saved Trajectory_of_Drone.mat with %d samples.\n', numel(t));
end
