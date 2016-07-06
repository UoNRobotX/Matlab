function [GridMap,y,yhat,oblen] = UpdateMap(GridMap,pose,rcQC,thQC,radii,camob,sen,map,height)
%This function utilises the camera data to form a approximate map.

%--------------------------------------------------------------------------
% Parameters

cameras = sen.cameras;
vision = sen.rad;
res = map.res;     
dec = map.dec;     
inc = map.inc;     
maxval = map.max;     
minval = map.min;
rad = vision/res;
MapY = zeros(size(GridMap,1),size(GridMap,2));

%--------------------------------------------------------------------------
% Rotation Matrices and Translation Vectors

rol = pose(4); pit = pose(5); yaw = pose(6);
rnBN = [pose(1);pose(2);cos(pit)*cos(rol)*height];                                                                                    
RnBN = [cos(yaw)*cos(pit),-sin(yaw)*cos(rol)+cos(yaw)*sin(pit)*sin(rol),...
sin(yaw)*sin(rol)+cos(yaw)*cos(rol)*sin(pit); sin(yaw)*cos(pit),...
cos(yaw)*cos(rol)+sin(rol)*sin(pit)*sin(yaw), ...
-cos(yaw)*sin(rol)+sin(yaw)*cos(rol)*sin(pit); ...
-sin(pit),cos(pit)*sin(rol),cos(pit)*cos(rol)];
rnCN = repmat(rnBN,1,cameras) + RnBN*sen.rbCB;
RnCN = RnBN*sen.RbCB;

%--------------------------------------------------------------------------
% Update Map

if ~isempty(rcQC)
    
    % Obtain Centre Points of Obstalces   
    obs = size(rcQC,2);
    ind = repmat(3*camob,3,1)-repmat([2,1,0]',1,obs);  
    rcPC = (repmat(radii./sin(thQC),3,1).*rcQC);
    rnPC = zeros(3,size(rcQC,2)); 
    for count = 1:obs
        rnPC(:,count) = RnCN(:,ind(:,count))*rcPC(:,count);
    end
    rnPN = rnPC + rnCN(:,camob);
    rnPN(3,:) = 0;
    
    % Get Index of Grid Cells Within Radius   
    gridpose = pose(1:2)./res + 1;
    lim = round([gridpose-rad,gridpose+rad]);
    xlim = max(1,min(lim(1,:),repmat(size(GridMap,1),1,2)));
    ylim = max(1,min(lim(2,:),repmat(size(GridMap,2),1,2)));
    xaxis = xlim(1):xlim(2); yaxis = ylim(1):ylim(2);
    indell = ceil((1:length(yaxis)*length(xaxis))./length(xaxis));
    gridcells = [repmat(xaxis,1,length(yaxis));yaxis(indell)];
    diff = gridcells - repmat(gridpose,1,size(gridcells,2));
    norms = sqrt(sum(diff.^2));
    radind = nonzeros((norms <= rad).*(1:size(gridcells,2)));
    cells = gridcells(:,radind);
      
    % Update Section of Maps     
    
    current = GridMap;
    scan = sub2ind(size(GridMap),cells(1,:),cells(2,:));
    GridMap(scan) = max(GridMap(scan)-dec,minval);
    rnGN = res.*[cells-0.5;zeros(1,size(cells,2))];
    for count = 1:obs
        rnPNmat = repmat(rnPN(:,count),1,size(rnGN,2));
        rcQCmat = repmat(rcQC(:,count),1,size(rnGN,2));
        rnCNmat = repmat(rnCN(:,camob(count)),1,size(rnGN,2));
        
        rcGC = (RnCN(:,ind(:,count)))'*(rnGN - rnCNmat);
        magnitude = sqrt(sum((rnPNmat-rnGN).^2));   
        circind = nonzeros((magnitude <= radii(count)).*(1:size(rnGN,2)));
        circle = sub2ind(size(GridMap),cells(1,circind),cells(2,circind));
        deccir = repmat(dec,1,length(circle));
        
        norms = repmat(sqrt(sum(rcGC.^2)),3,1);
        dotprod = sum(rcQCmat.*(rcGC./norms));
        thGC = abs(acos(dotprod));     
        ellbound = (thGC <= abs(thQC(count))).*(dotprod >= 0);   
        ellind = nonzeros(ellbound.*(1:size(rnGN,2)));    
        ell = sub2ind(size(GridMap),cells(1,ellind),cells(2,ellind));
        ellipse = ell(~ismember(ell,circle));
        decell = repmat(dec,1,length(ellipse));
        
        change = (current(ellipse) - GridMap(ellipse)) ~= dec;
        indell = nonzeros(change.*(1:length(ellipse)));    
        if ~isempty(indell)
            decell(indell) = 0;
        end
        change = (current(circle) - GridMap(circle)) ~= dec;
        indcir = nonzeros(change.*(1:length(circle))); 
        if ~isempty(indcir)
            deccir(indcir) = 0;
        end
        GridMap(ellipse) = max(GridMap(ellipse) + decell,minval); 
        GridMap(circle) = min(GridMap(circle) + inc + deccir,maxval);
    end 
    MapY = GridMap - current;  
    
end

% [y,yhat] = ObMeasurements(MapY,GridMap,map,rnBN,Limit);
% oblen = length(y);

end