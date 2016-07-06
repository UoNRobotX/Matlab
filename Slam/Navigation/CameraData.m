function [rcQC,thQC,camobs,radii,rcLC,thLC,camlnd,lndid] = CameraData(pose,sen,obs,height,lndmks)
%This function Collects all Necessary Camera Data for Map Updation

%--------------------------------------------------------------------------
% Parameters

cameras = size(sen.rbCB,2);
vision = sen.rad; deg = sen.deg;  
rol = pose(4); pit = pose(5); yaw = pose(6);
camobs = []; radii = []; camlnd = []; lndid = [];
rcQC = zeros(3,cameras*(length(obs)));
thQC = zeros(1,cameras*(length(obs)));        
rcLC = zeros(3,cameras*(length(lndmks))); 
thLC = zeros(1,cameras*(length(lndmks))); 

%--------------------------------------------------------------------------
% Rotation Matrices and Translation Vectors

rnBN = [pose(1);pose(2);cos(pit)*cos(rol)*height];                                                                                    
RnBN = [cos(yaw)*cos(pit),-sin(yaw)*cos(rol)+cos(yaw)*sin(pit)*sin(rol),...
sin(yaw)*sin(rol)+cos(yaw)*cos(rol)*sin(pit); sin(yaw)*cos(pit),...
cos(yaw)*cos(rol)+sin(rol)*sin(pit)*sin(yaw), ...
-cos(yaw)*sin(rol)+sin(yaw)*cos(rol)*sin(pit); ...
-sin(pit),cos(pit)*sin(rol),cos(pit)*cos(rol)];
rnCN = repmat(rnBN,1,cameras) + RnBN*sen.rbCB;
RnCN = RnBN*sen.RbCB;

%--------------------------------------------------------------------------
% Obtain Information form Each Camera Image

okey = 1; lkey = 1;
obsind = 1:size(obs,2); lndind = 1:size(lndmks,2);
for count = 1:cameras     
    rotind = (3*count);
    trans = rnCN(:,count);
    rot = RnCN(:,rotind-2:rotind)';
    ang = deg(count)/2; 
    delta = obs-repmat(trans,1,size(obs,2));
    local = rot*delta;
    bearing = atan2(local(1,:),local(3,:));
    norms = sqrt(sum(local.^2));
    ind = nonzeros((norms <= vision).*(abs(bearing) <= ang).*obsind);
    if ~isempty(ind)
        onum = length(ind);
        camobs = cat(2,camobs,repmat(count,1,onum));
        radii = cat(2,radii,-obs(3,ind));
        rcQC(:,okey:okey+onum-1) = (local(:,ind))./repmat(norms(ind),3,1);
        thQC(:,okey:okey+onum-1) = radii(okey:okey+onum-1)./norms(ind);
        okey = okey + onum;
    end     
    delta = lndmks-repmat(trans,1,size(lndmks,2));  
    trans = rot*delta;                         
    bearing = atan2(trans(1,:),trans(3,:));                  
    norms = sqrt(sum(trans.^2));   
    ind = nonzeros((norms <= vision).*(abs(bearing) <= ang).*lndind); 
    if ~isempty(ind)
        lnum = length(ind);
        camlnd = cat(2,camlnd,repmat(count,1,lnum));
        lndid = cat(1,lndid,ind);
        rcLC(:,lkey:lkey+lnum-1) = (trans(:,ind))./repmat(norms(ind),3,1);
        thLC(:,lkey:lkey+lnum-1) = -lndmks(3,ind)./norms(ind);
        lkey = lkey + lnum;
    end
end
rcQC = rcQC(:,any(rcQC,1));  
thQC = thQC(:,any(thQC,1));  
rcLC = rcLC(:,any(rcLC,1)); 
thLC = thLC(:,any(thLC,1)); 

end