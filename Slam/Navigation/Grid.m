function [gridnew,gridy] = Grid(grid,rnCN,RnCN,QC,cam,rad,scan,cells,map)
% This function updates the virtual grid which retaines all history,
% and the grid for the current obstacle detection

res = map.res; dec = map.dec; inc = map.inc;     
maxval = map.max; minval = map.min;
rcQC = QC(1:3,:); thQC = QC(4,:);

%--------------------------------------------------------------------------
% Obtain Central Point of Obstalces and Make Down Zero
  
obs = size(rcQC,2); 
if obs ~= 0
    rnPC = zeros(3,obs); 
    cind = repmat(3*cam,3,1)-repmat([2,1,0]',1,obs);
    rcPC = (repmat(rad./sin(thQC),3,1).*rcQC);
    for count = 1:obs
        rnPC(:,count) = RnCN(:,cind(:,count))*rcPC(:,count);
    end
    rnPN = rnPC + rnCN(:,cam); 
    rnPN(3,:) = 0;
end

%--------------------------------------------------------------------------
% Update Grids Via Decrease all, Increase in Circle, Constant in Ellipse
     
gridnew = grid;
rnGN = res.*[cells-0.5;zeros(1,size(cells,2))];
len = size(rnGN,2);
free = ones(1,length(scan));
for count = 1:obs    
    rnPNmat = repmat(rnPN(:,count),1,len);
    rcQCmat = repmat(rcQC(:,count),1,len);
    rnCNmat = repmat(rnCN(:,cam(count)),1,len);
    rcGC = (RnCN(:,cind(:,count))')*(rnGN - rnCNmat);
    
    % Obstacle Circle   
    magnitude = sqrt(sum((rnPNmat-rnGN).^2));   
    indcir = nonzeros((magnitude <= rad(count)).*(1:len)); % Index of all cells within obstalce radius
    if ~isempty(indcir) 
        circle = sub2ind(size(gridnew),cells(1,indcir),cells(2,indcir));    
        gridnew(circle) = gridnew(circle) + inc;
        free(indcir) = 0;
    end
           
    % Shadow Ellipse   
    norms = sqrt(sum(rcGC.^2));
    dotprod = sum(rcQCmat.*rcGC)./norms; thGC = acos(dotprod); 
    bound = (thGC <= thQC(count)).*(dotprod >= 0);      % Bounds of Ellipse
    indell = nonzeros(bound.*(1:len));
    if ~isempty(indell)
        free(indell) = 0;
    end   
end
decrement = scan(nonzeros(free.*(1:numel(scan))));
gridnew(decrement) = gridnew(decrement) - dec;
gridy = gridnew - grid;
gridnew = max(gridnew,minval);
gridnew = min(gridnew,maxval);

end