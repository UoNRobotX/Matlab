function [y,C,D,R] = MeasurementModel(x,~,param)
% Boat measurement model

% Lengths of all the parameters if wanted to be seperated
yobs = param.oblen;
yland = param.landlen;

% Set variances.
Rgps = param.gpsnoise^2;
Rimu = param.imunoise^2;
Rgrid = param.gridnoise^2;
Rbear = param.bnoise^2;
Rrange = param.rnoise^2;

% Estimated GPS reading and Linear measurement model.
y = [x(1);x(2);x(3);x(4);x(5);x(6)];
Rdiag = [Rgps,Rgps,Rgps,Rimu,Rimu,Rimu];
if ~isempty(param.yobhat)
    y = cat(1,y,param.yobhat);
    Rdiag = cat(2,Rdiag,Rgrid*ones(1,yobs));
end
if ~isempty(param.ylandhat)
    for count = 1:yland/2
        y = cat(1,y,param.ylandhat(count:count+1));
        Rdiag = cat(2,Rdiag,[Rbear,Rrange]);
    end
end

C = [];
D = [];

% Covariance of additive measurement noise
R = diag(Rdiag);
end