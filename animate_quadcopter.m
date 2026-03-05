function animate_quadcopter(t, X, traj, trajectory, P)
% ANIMATE_QUADCOPTER  
% -------------------------------------------------------------
% 3D animation of quadrotor motion and orientation
% Consistent with model/notation in the Quadrotor LQR paper.
%
% INPUTS:
%   t   : 1xN time vector
%   X   : 12xN state history
%   traj: reference trajectory (for plotting path)
%   trajectory : string (circle, helix, lemniscate...)
%   P   : quadrotor parameters (for arm length)
% -------------------------------------------------------------

N = numel(t);

pos = X(1:3,:);       % (x,y,z)
eul = X(7:9,:);       % (phi,theta,psi)

% Arm length from quad_params (consistent with dynamics)
L = P.L;

%% ============================================================
% Create results folder
% ============================================================
if ~exist('results','dir')
    mkdir('results');
end

%% ============================================================
% Setup figure
% ============================================================
f = figure('Color','w','Name','Quadrotor Animation','Position',[200 200 1100 700]);
ax = axes('Parent',f);
grid(ax,'on'); hold(ax,'on'); axis(ax,'equal');

xlabel(ax,'X (m)'); 
ylabel(ax,'Y (m)'); 
zlabel(ax,'Z (m)');
view(ax, 40, 25);

% World limits
allpts = pos';
mins = min(allpts) - 1.5;
maxs = max(allpts) + 1.5;

xlim(ax, [mins(1) maxs(1)]);
ylim(ax, [mins(2) maxs(2)]);
zlim(ax, [0 max(2, maxs(3))]);

%% Reference Path
if nargin >= 3 && isfield(traj,'p_ref')
    plot3(ax, traj.p_ref(:,1), traj.p_ref(:,2), traj.p_ref(:,3), ...
        '--','Color',[0.2 0.2 0.2],'LineWidth',1);
end

%% Drone body graphics
arm1_line = line(ax,'LineWidth',3,'Color','b'); % X-arm
arm2_line = line(ax,'LineWidth',3,'Color','r'); % Y-arm
trail     = animatedline(ax,'LineWidth',1.5,'Color','k');

%% GIF setup
gifname = fullfile('results', ['animation_' trajectory '.gif']);
gif_first = true;

%% ============================================================
% Animation Loop
% ============================================================
for k = 1:2:N
    pk = pos(:,k);
    R = eul2rotm_zyx(eul(:,k)); % local function below

    % Arm vectors in body frame
    armX = [ -L  L;   0  0;   0  0 ];
    armY = [  0  0;  -L  L;   0  0 ];

    % Transform to world frame
    armX_w = R * armX + pk;
    armY_w = R * armY + pk;

    % Update arm visuals
    set(arm1_line, 'XData', armX_w(1,:), 'YData', armX_w(2,:), 'ZData', armX_w(3,:));
    set(arm2_line, 'XData', armY_w(1,:), 'YData', armY_w(2,:), 'ZData', armY_w(3,:));

    % Add trajectory trail
    addpoints(trail, pk(1), pk(2), pk(3));

    % Smooth camera follow
    campos(ax, pk' + [8 8 5]);

    drawnow;
    pause(0.01);

    % Save as GIF
    frame = getframe(f);
    [A,map] = rgb2ind(frame2im(frame),256);
    if gif_first
        imwrite(A,map,gifname,'gif','LoopCount',Inf,'DelayTime',0.02);
        gif_first = false;
    else
        imwrite(A,map,gifname,'gif','WriteMode','append','DelayTime',0.02);
    end
end

%% Save final PNG
saveas(f, fullfile('results', ['figure_' trajectory '.png']));

end

%% ============================================================
% Helper: ZYX rotation matrix (phi,theta,psi)
% ------------------------------------------------------------
function R = eul2rotm_zyx(eul)
phi = eul(1);    theta = eul(2);    psi = eul(3);

c1=cos(psi); s1=sin(psi);
c2=cos(theta); s2=sin(theta);
c3=cos(phi); s3=sin(phi);

Rz = [c1 -s1 0; s1 c1 0; 0 0 1];
Ry = [c2 0 s2; 0 1 0; -s2 0 c2];
Rx = [1 0 0; 0 c3 -s3; 0 s3 c3];

R = Rz * Ry * Rx;
end
