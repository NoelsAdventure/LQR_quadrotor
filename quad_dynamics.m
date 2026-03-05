function dX = quad_dynamics(~, X, F, P)
% QUAD_DYNAMICS
% -------------------------------------------------------------
% Full nonlinear quadrotor dynamics used in the paper:
%
% State:
%   X = [x y z  vx vy vz  phi theta psi  p q r]'
%
% Input:
%   F = [f1 f2 f3 f4]'  rotor forces
%
% Mixer:   u = D * F
% where u = [ft; tau_phi; tau_theta; tau_psi]
% -------------------------------------------------------------

%% Extract states
vx = X(4);    vy = X(5);    vz = X(6);
phi   = X(7); theta = X(8); psi = X(9);
p     = X(10); q = X(11);    r = X(12);

%% Mixer matrix (+ configuration), from Eq. (1) of paper
L = P.L;
km_kf = P.km / P.kf;

D = [ 1      1      1      1   ;
      0     -L      0      L   ;
     -L      0      L      0   ;
     -km_kf  km_kf -km_kf  km_kf ];

%% Body wrench
u = D * F;
ft  = u(1);
tau = u(2:4);

%% Rotation matrix R (body → world), ZYX convention
cphi = cos(phi);   sphi = sin(phi);
cth  = cos(theta); sth = sin(theta);
cpsi = cos(psi);   spsi = sin(psi);

R = [ cth*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi;
      cth*spsi,  sphi*sth*spsi + cphi*cpsi,  cphi*sth*spsi - sphi*cpsi;
     -sth,       sphi*cth,                 cphi*cth ];

%% Translational dynamics
p_dot = [vx; vy; vz];
v_dot = (1/P.m) * (R * [0;0;ft]) - [0;0;P.g];

%% Euler angle kinematics
E = [1,  sphi*tan(theta),  cphi*tan(theta);
     0,  cphi,           -sphi;
     0,  sphi/cth,        cphi/cth];

eul_dot = E * [p; q; r];

%% Rotational dynamics
I = P.I;
omega = [p; q; r];
omega_dot = I \ (tau - cross(omega, I*omega));

%% Complete state derivative
dX = [p_dot; v_dot; eul_dot; omega_dot];

end
