function [gridm,ym,ymhat,gridc,yc,ychat,ctime,mtime] = Update(gridm,...
gridc,est,obsinfo,lndinfo,obscam,lndcam,lmrks,sen,map)
% Thus function updates the virtual map whilst obtaining the current sensor
% data and approximated map data

%--------------------------------------------------------------------------
% C++ Version

ocam = (obscam-1)'; lcam = (lndcam-1)'; 
linfo = lndinfo; linfo(5,:) = linfo(5,:) -1;
mparams = [map.res,map.dec,map.inc,map.thres,map.max,map.min];
sparams = [sen.rad,sen.height]; cmark = [lmrks;-lmrks(3,:)]; tic;
[gridc,yc,ychat] = SLAM(est,ocam,lcam,obsinfo,linfo,gridc,cmark,...
sen.rbCB,sen.RbCB,mparams,sparams);
ctime = toc;

%--------------------------------------------------------------------------
% Matlab Version

tic;
res = map.res; 
camera = sen.camera; 
radgrid = sen.rad/res; 

%--------------------------------------------------------------------------
% Rotation Matrices and Translation Vectors

rol = est(4); pit = est(5); yaw = est(6);
rnBN = [est(1);est(2);cos(pit)*cos(rol)*sen.height];                                                     
RnBN = [cos(yaw)*cos(pit),-sin(yaw)*cos(rol)+cos(yaw)*sin(pit)*sin(rol),...
sin(yaw)*sin(rol)+cos(yaw)*cos(rol)*sin(pit); sin(yaw)*cos(pit),...
cos(yaw)*cos(rol)+sin(rol)*sin(pit)*sin(yaw), ...
-cos(yaw)*sin(rol)+sin(yaw)*cos(rol)*sin(pit); ...
-sin(pit),cos(pit)*sin(rol),cos(pit)*cos(rol)];
rnCN = repmat(rnBN,1,camera) + RnBN*sen.rbCB;
RnCN = RnBN*sen.RbCB;

%--------------------------------------------------------------------------
% Obtain Index of Grid Cells Within Radius

gridpose = est(1:2)./res + 1;
lim = round([gridpose-radgrid,gridpose+radgrid]);
nlim = max(1,min(lim(1,:),repmat(size(gridm,1),1,2)));
elim = max(1,min(lim(2,:),repmat(size(gridm,2),1,2)));
naxis = nlim(1):nlim(2); eaxis = elim(1):elim(2);
indell = ceil((1:length(eaxis)*length(naxis))./length(naxis));
gridcells = [repmat(naxis,1,length(eaxis));eaxis(indell)];
diff = gridcells - repmat(gridpose,1,size(gridcells,2));
norms = sqrt(sum(diff.^2));
radind = nonzeros((norms <= radgrid).*(1:size(gridcells,2)));
cells = gridcells(:,radind);
scan = sub2ind(size(gridm),cells(1,:),cells(2,:));

%--------------------------------------------------------------------------
% Update Grids if Detected Obstacles, Obtain Obstacle & Landmark Data

[gridm,gridy] = Grid(gridm,rnCN,RnCN,obsinfo,obscam,scan,cells,map);
[yo,yohat] = ObsVect(gridy,gridm,map,rnBN(1:2,:),scan,sen.rad);
[yl,ylhat] = LandVect(rnCN,RnCN,rnBN,lndinfo,lndcam,lmrks);
ym = [yo;yl];  ymhat = [yohat;ylhat];
mtime = toc;

end