function [y,R] = Measure(x,~,yhat,param)
% Measurement Model for RobotX Platform

%--------------------------------------------------------------------------
% Form Covariance Matrix

Rcam = [];
Rdreck = [param.gnoise,param.inoise];
Rgrid = repmat(param.mnoise,1,360);
lmrk = (length(yhat)-360)/2;
if lmrk ~= 0
    index = sort(repmat([1,2],1,lmrk));
    Rcam = param.cnoise(index);
end
R = diag([Rdreck,Rgrid,Rcam]);

%--------------------------------------------------------------------------
% Output Vector

y = [x(1:6);yhat];

end