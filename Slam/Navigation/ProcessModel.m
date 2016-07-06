function [statenext,A,B,Q] = ProcessModel(state,~,param)
% Process Model

T = param.T;
eta = state(1:6);
nu = state(7:12);
phi = eta(4);
theta = eta(5);
psi = eta(6);

% Rotation matrix.
R = [ cos(psi)*cos(theta) -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi) sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta)  ;
      sin(psi)*cos(theta) cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi)  -cos(psi)*sin(phi)+sin(psi)*cos(phi)*sin(theta) ;
      -sin(theta)         cos(theta)*sin(phi)                             cos(theta)*cos(phi)                             ];
  
Tr = [1 sin(phi)*tan(theta) cos(phi)*tan(theta) ;
      0 cos(phi)            -sin(phi)           ;
      0 sin(phi)/cos(theta) cos(phi)/cos(theta) ];
  
J = [ R zeros(size(R)) ; zeros(size(Tr)) Tr ];

etadot = J*nu;

% This is the Euler discretisation
etanext = eta + T*etadot;
statenext = [etanext;nu];

% Linear process model, Not required for UKF
A = [];
B = [];

% Covariance of additive process noise
Q = diag(param.Q);
end