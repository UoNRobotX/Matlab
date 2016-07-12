function [state,Q] = Process(state,~,param) 
% Process Model for RobotX Platform

time = param.time;
eta = state(1:6);
nu = state(7:12);

%--------------------------------------------------------------------------
% Obtain Rotation Matrix

rol = eta(4); pit = eta(5); yaw = eta(6);
R = [cos(yaw)*cos(pit),-sin(yaw)*cos(rol)+cos(yaw)*sin(pit)*sin(rol),...
sin(yaw)*sin(rol)+cos(yaw)*cos(rol)*sin(pit); sin(yaw)*cos(pit),...
cos(yaw)*cos(rol)+sin(rol)*sin(pit)*sin(yaw), ...
-cos(yaw)*sin(rol)+sin(yaw)*cos(rol)*sin(pit); ...
-sin(pit),cos(pit)*sin(rol),cos(pit)*cos(rol)];

%--------------------------------------------------------------------------
% Equate Time Derivative
  
Tr = [1,sin(rol)*tan(pit),cos(rol)*tan(pit);0,cos(rol),-sin(rol);0,...
sin(rol)/cos(pit),cos(rol)/cos(pit)];  
J = [R,zeros(size(R));zeros(size(Tr)),Tr];
etadot = J*nu;

%--------------------------------------------------------------------------
% Euler Discretisation

eta = eta + time*etadot;
state = [eta;nu];

%--------------------------------------------------------------------------
% Covariance

Q = diag(param.Q);

end