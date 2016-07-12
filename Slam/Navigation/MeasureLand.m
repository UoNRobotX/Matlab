function [y,yhat] = MeasureLand(rnCN,RnCN,rnBN,RnBN,LC,cam,lmrks)
% This function returns landmark bearings and ranges, both from 
% the camera coordinates and dead reckoning

bearingy = []; rangey = [];
bearingyhat = []; rangeyhat = [];

%--------------------------------------------------------------------------
% Camera Observation

if ~isempty(LC)
    rcLC = LC(1:3,:);
    thLC = LC(4,:);
    lnd = size(rcLC,2);
    ind = repmat(3*cam,3,1)-repmat([2,1,0]',1,lnd);
    rcPC = repmat((-lmrks(3,:)./sin(thLC)),3,1).*rcLC;
    rnPC = zeros(3,lnd);
    for count = 1:lnd
        rnPC(:,count) = RnCN(:,ind(:,count))*rcPC(:,count);
    end
    rnPN = rnPC + rnCN(:,cam);
    diff = rnPN - repmat(rnBN,1,lnd);
    bearingy = atan2(diff(2,:),diff(1,:));
    rangey = sqrt(sum(diff.^2));
end

%--------------------------------------------------------------------------
% GPS and IMU Approximation with Assumed GPS Landmark Location

if ~isempty(lmrks)
    local = RnBN*(lmrks - repmat(rnBN,1,size(lmrks,2)));
    bearingyhat = atan2(local(2,:),local(1,:));
    rangeyhat = sqrt(sum(local.^2));  
end

%--------------------------------------------------------------------------
% Form Outputs

y = [rangey,bearingy]';
yhat = [rangeyhat,bearingyhat]';

end