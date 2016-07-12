function [QC,LC,rad,ocam,lcam,lndid] = Camera(state,sen,boat,obs,lmrks)
%This function collects all necessary Camera Data for Map Updation

vision = sen.rad; deg = sen.fov; camera = sen.camera; 
rol = state(4); pit = state(5); yaw = state(6);
ocam = []; rad = []; lcam = []; lndid = [];
QC = zeros(4,camera*(length(obs))); LC = zeros(4,camera*(length(lmrks))); 

%--------------------------------------------------------------------------
% Rotation Matrices and Translation Vectors

rnBN = [state(1);state(2);cos(pit)*cos(rol)*boat.height];                                             
RnBN = [cos(yaw)*cos(pit),-sin(yaw)*cos(rol)+cos(yaw)*sin(pit)*sin(rol),...
sin(yaw)*sin(rol)+cos(yaw)*cos(rol)*sin(pit); sin(yaw)*cos(pit),...
cos(yaw)*cos(rol)+sin(rol)*sin(pit)*sin(yaw), ...
-cos(yaw)*sin(rol)+sin(yaw)*cos(rol)*sin(pit); ...
-sin(pit),cos(pit)*sin(rol),cos(pit)*cos(rol)];
rnCN = repmat(rnBN,1,camera) + RnBN*sen.rbCB;
RnCN = RnBN*sen.RbCB;

%--------------------------------------------------------------------------
% Obtain Noisy Information form Each Camera Image

noise =  sqrt(sen.cnoise([1,2])');
oid = 1; obsind = 1:size(obs,2);
lid = 1; lndind = 1:size(lmrks,2);
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
        ocam = cat(2,ocam,repmat(count,1,num)); 
        rad = cat(2,rad,-obs(3,ind)); pit = atan2(loc(2,ind),loc(3,ind));
        rnoise = norms(ind).*cos(pit) + noise(1).*randn(1,num);
        bnoise = bearing(ind) + noise(2).*randn(1,num);
        noiseloc = [rnoise.*sin(bnoise);loc(2,ind);rnoise.*cos(bnoise)];  
        mag = sqrt(sum(noiseloc.^2));
        QC(1:3,oid:oid+num-1) = noiseloc./repmat(mag,3,1);
        QC(4,oid:oid+num-1) = asin(rad(oid:oid+num-1)./mag);
        oid = oid + num;
    end  
    loc = rot*(lmrks-repmat(trans,1,size(lmrks,2)));
    norms = sqrt(sum(loc.^2));
    bearing = atan2(loc(1,:),loc(3,:));   
    ind = nonzeros((norms <= vision).*(abs(bearing) <= ang).*lndind);
    if ~isempty(ind)
        num = length(ind);
        lcam = cat(2,lcam,repmat(count,1,num)); 
        lndid = cat(1,lndid,ind);  pit = atan2(loc(3,ind),loc(2,ind));
        rnoise = norms(ind).*sin(pit) + noise(1).*randn(1,num);
        bnoise = bearing(ind) + noise(2).*randn(1,num);
        noiseloc = [rnoise.*sin(bnoise);loc(2,ind);rnoise.*cos(bnoise)];  
        mag = sqrt(sum(noiseloc.^2));
        LC(1:3,lid:lid+num-1) = noiseloc./repmat(mag,3,1);
        LC(4,lid:lid+num-1) = asin(-lmrks(3,ind)./mag);
        lid = lid + num;
    end     
end
QC = QC(:,any(QC,1));
LC = LC(:,any(LC,1));

end