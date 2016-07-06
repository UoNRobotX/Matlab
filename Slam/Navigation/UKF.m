function [x,P] = UKF(P,x,u,y,processModel,measurementModel,param,stateConstraint)

%
% Input arguments
%
% x: current state        (n x 1)
% u: current input        (m x 1)
% y: current measurement  (p x 1)
%
% P: estimation error covariance (n x n)
%
% processModel:     function handle to process model
% measurementModel: function handle to measurement model
% param:            parameters to pass to process and measurement models
% stateConstraint:  function handle to state constraints (optional)
%

pFunc = @(x) processModel(x,u,param);
mFunc = @(x) measurementModel(x,u,param);
if nargin >= 8
    cFunc = @(x) stateConstraint(x,param);
else
    cFunc = @(x) x;
end

% Retrieve process and measurement covariance for the current
% state and input
[~,~,~,Q] = processModel(x,u,param);
[~,~,~,R] = measurementModel(x,u,param);

%
% Apply unscented transform through process model to obtain
% a priori state estimate and covariance
%
[x,P,sigma] = UT(x,P,pFunc,length(x),cFunc);

P = P + Q;          % Add process uncertainty

%
% Apply unscented transform through the measurement model to obtain
% measurement covariance and cross covariance between state and measurement
%
[yhat,Py,~,Pxy] = UT(x,P,mFunc,length(y),cFunc, ...
    sigma ...              % Re-use existing sigma points
    );

Py = Py + R;        % Add measurement uncertainty

%
% a posteriori state estimate and covariance
%
K = Pxy/Py;
x = x + K*(y - yhat);
P = P - K*Py*K.';

% Apply constraints
if nargin >= 8
    x = stateConstraint(x,param);
end
