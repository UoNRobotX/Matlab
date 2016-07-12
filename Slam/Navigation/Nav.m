function [grid,y,yht] = Nav(grid,est,QC,LC,ocam,lcam,rad,lmrk,sen,hght,map)
% Thus function updates the virtual map whilst obtaining the current sensor
% data and approximated map data

camera = sen.camera; 
res = map.res; radgrid = sen.rad/res;

%--------------------------------------------------------------------------
% Rotation Matrices and Translation Vectors

rol = est(4); pit = est(5); yaw = est(6);
rnBN = [est(1);est(2);cos(pit)*cos(rol)*hght];                                                     
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
xlim = max(1,min(lim(1,:),repmat(size(grid,1),1,2)));
ylim = max(1,min(lim(2,:),repmat(size(grid,2),1,2)));
xaxis = xlim(1):xlim(2); yaxis = ylim(1):ylim(2);
indell = ceil((1:length(yaxis)*length(xaxis))./length(xaxis));
gridcells = [repmat(xaxis,1,length(yaxis));yaxis(indell)];
diff = gridcells - repmat(gridpose,1,size(gridcells,2));
norms = sqrt(sum(diff.^2));
radind = nonzeros((norms <= radgrid).*(1:size(gridcells,2)));
cells = gridcells(:,radind);
scan = sub2ind(size(grid),cells(1,:),cells(2,:));

%--------------------------------------------------------------------------
% Update Grids if Detected Obstacles, Obtain Obstacle & Landmark Data

[grid,gridy] = Grid(grid,rnCN,RnCN,QC,ocam,rad,scan,cells,map); 
[yo,yhto] = MeasureObs(gridy,grid,map,rnBN(1:2,:),scan);
[yl,yhtl] = MeasureLand(rnCN,RnCN,rnBN,RnBN,LC,lcam,lmrk);
y = [yo;yl]; 
yht = [yhto;yhtl];

end