function [obsinfo,lndinfo,obscam,lndcam] = Camera(state,sen,obs,lmrks)
%This function collects all necessary Camera Data for Map Updation

vision = sen.rad; deg = sen.fov; camera = sen.camera; 
rol = state(4); pit = state(5); yaw = state(6);
obsinfo = zeros(5,camera*(length(obs))); 
lndinfo = zeros(5,camera*(length(lmrks))); 
obscam = []; lndcam = [];

%--------------------------------------------------------------------------
% Rotation Matrices and Translation Vectors

rnBN = [state(1);state(2);cos(pit)*cos(rol)*sen.height];                                             
RnBN = [cos(yaw)*cos(pit),-sin(yaw)*cos(rol)+cos(yaw)*sin(pit)*sin(rol),...
sin(yaw)*sin(rol)+cos(yaw)*cos(rol)*sin(pit); sin(yaw)*cos(pit),...
cos(yaw)*cos(rol)+sin(rol)*sin(pit)*sin(yaw), ...
-cos(yaw)*sin(rol)+sin(yaw)*cos(rol)*sin(pit); ...
-sin(pit),cos(pit)*sin(rol),cos(pit)*cos(rol)];
rnCN = repmat(rnBN,1,camera) + RnBN*sen.rbCB;
RnCN = RnBN*sen.RbCB;

%--------------------------------------------------------------------------
% Obtain Noisy Information form Each Camera Image

oindex = 1; obsind = 1:size(obs,2);
lindex = 1; lndind = 1:size(lmrks,2);
for count = 1:camera     
    trans = rnCN(:,count);
    rot = RnCN(:,(3*count)-2:(3*count))';
    ang = deg(count)/2; 
    loc = rot*(obs-repmat(trans,1,size(obs,2)));
    norms = sqrt(sum(loc.^2));
    bearing = atan2(loc(1,:),loc(3,:)); 
    ind = nonzeros((norms <= vision).*(abs(bearing) <= ang).*obsind);  
    if ~isempty(ind)
        num = length(ind);
        arr = oindex:oindex+num-1;
        obscam = cat(2,obscam,repmat(count,1,num)); 
        mag = sqrt(sum(loc(:,ind).^2));
        obsinfo(1:3,arr) = loc(:,ind)./repmat(mag,3,1);
        obsinfo(4,arr) = asin(-obs(3,ind)./mag);
        obsinfo(5,arr) = -obs(3,ind); 
        oindex = oindex + num;
    end  
    loc = rot*(lmrks-repmat(trans,1,size(lmrks,2)));
    norms = sqrt(sum(loc.^2));
    bearing = atan2(loc(1,:),loc(3,:));   
    ind = nonzeros((norms <= vision).*(abs(bearing) <= ang).*lndind);
    if ~isempty(ind)
        num = length(ind);
        arr = lindex:lindex+num-1;
        lndcam = cat(2,lndcam,repmat(count,1,num)); 
        mag = sqrt(sum(loc(:,ind).^2));        
        lndinfo(1:3,arr) = loc(:,ind)./repmat(mag,3,1);
        lndinfo(4,arr) = asin(-lmrks(3,ind)./mag);
        lndinfo(5,arr) = ind;
        lindex = lindex + num;
    end     
end
obsinfo = obsinfo(:,any(obsinfo,1));
lndinfo = lndinfo(:,any(lndinfo,1));

end