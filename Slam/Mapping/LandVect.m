function [y,yhat] = LandVect(rnCN,RnCN,rnBN,lndinfo,lndcam,lmrks)
% This function returns landmark bearings and ranges, both from 
% the camera coordinates and dead reckoning

angley = []; rangey = [];
angleyhat = []; rangeyhat = [];

%--------------------------------------------------------------------------
% Obtain Bearing and Ranges

if ~isempty(lndinfo)
    rcLC = lndinfo(1:3,:);
    thLC = lndinfo(4,:);
    signature = lndinfo(5,:);

    % Camera Observation
    rnPC = zeros(3,size(rcLC,2));
    rcPC = repmat((-lmrks(3,signature)./sin(thLC)),3,1).*rcLC;
    for count = 1:size(rcLC,2)
        index = (3*lndcam(count)-2):(3*lndcam(count));
        rnPC(:,count) = RnCN(:,index)*rcPC(:,count);
    end
    rnPN = rnPC + rnCN(:,lndcam);
    diff = rnPN - repmat(rnBN,1,size(rcLC,2));
    rangey = sqrt(sum(diff.^2));
    angley = atan2(diff(2,:),diff(1,:));    
    angley(angley < 0 ) = angley(angley < 0 ) + 2*pi; 
    angley(angley >= 2*pi) = angley(angley >= 2*pi) - 2*pi;
    
    % 'True' Observations from Known NED Landmark Locations 
    diff = lmrks(:,signature) - repmat(rnBN,1,size(rcLC,2));
    rangeyhat = sqrt(sum(diff.^2)); 
    angleyhat = atan2(diff(2,:),diff(1,:));
    angleyhat(angleyhat < 0 ) = angleyhat(angleyhat < 0 ) + 2*pi; 
    angleyhat(angleyhat >= 2*pi) = angleyhat(angleyhat >= 2*pi) - 2*pi;
end   

%--------------------------------------------------------------------------
% Form Outputs

y = [rangey,angley]';
yhat = [rangeyhat,angleyhat]';

end