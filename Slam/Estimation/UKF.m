function [x,Px] = UKF(x,y,yhat,u,process,measure,param)

pfunc = @(x) process(x,u,param);
mfunc = @(x) measure(x,u,yhat,param);

%--------------------------------------------------------------------------
% Obtain Covariance Matrices for Process and Measurement Models

[~,Q] = process(x,u,param);
[~,R] = measure(x,u,yhat,param);

%--------------------------------------------------------------------------
% Apply Unscented Transform through Process Model to Obtain a Priori State
% Estimate and Covariance

[x,Px,xsigma,~] = Unscented(x,param.P,pfunc,length(x),[]);
Px = Px + Q; 

%--------------------------------------------------------------------------
% Apply Unscented Transform through Measurement Model to Obtain
% Measurement Covariance and Cross Covariance Between State and 
% Measurement

[yhat,Py,~,Pxy] = Unscented(x,Px,mfunc,length(y),xsigma);
Py = Py + R;

%--------------------------------------------------------------------------
% A Posteriori State Estimate and Covariance

K = Pxy/Py;
x = x + K*(y - yhat);
Px = Px - K*Py*K.';