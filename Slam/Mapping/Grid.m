function [grid,gridy] = Grid(gridc,rnCN,RnCN,obsinfo,obscam,scan,cells,map)
% This function updates the virtual grid which retaines all history,
% and the grid for the current obstacle detection

maxval = map.max; minval = map.min;
res = map.res; dec = map.dec; inc = map.inc;     
rcQC = obsinfo(1:3,:); thQC = obsinfo(4,:); rad = obsinfo(5,:);

%--------------------------------------------------------------------------
% Obtain Central Point of Obstalces and Make Down Zero
  
obs = size(rcQC,2); 
if obs ~= 0
    rnPC = zeros(3,obs); 
    rcPC = (repmat(rad./sin(thQC),3,1).*rcQC);
    for count = 1:obs
        index = (3*obscam(count)-2):(3*obscam(count));
        rnPC(:,count) = RnCN(:,index)*rcPC(:,count);
    end
    rnPN = rnPC + rnCN(:,obscam); 
    rnPN(3,:) = 0;
end

%--------------------------------------------------------------------------
% Update Grids Via Decrease all, Increase in Circle, Constant in Ellipse
     
grid = gridc;
rnGN = res.*[cells-0.5;zeros(1,size(cells,2))];
len = size(rnGN,2);
free = ones(1,length(scan));
for count = 1:obs    
    index = (3*obscam(count)-2):(3*obscam(count));
    rnPNmat = repmat(rnPN(:,count),1,len);
    rcQCmat = repmat(rcQC(:,count),1,len);
    rnCNmat = repmat(rnCN(:,obscam(count)),1,len);
    rcGC = (RnCN(:,index)')*(rnGN - rnCNmat);
    
    % Obstacle Circle   
    magnitude = sqrt(sum((rnPNmat-rnGN).^2));   
    indcir = nonzeros((magnitude <= rad(count)).*(1:len));
    if ~isempty(indcir) 
        circle = sub2ind(size(grid),cells(1,indcir),cells(2,indcir));    
        grid(circle) = grid(circle) + inc;
        free(indcir) = 0;
    end
           
    % Shadow Ellipse   
    norms = sqrt(sum(rcGC.^2));
    dotprod = sum(rcQCmat.*rcGC)./norms;
    thGC = acos(dotprod);
    bound = (thGC <= thQC(count)).*(dotprod >= 0);
    indell = nonzeros(bound.*(1:len));
    if ~isempty(indell)
        free(indell) = 0;
    end   
end
gridy = grid - gridc;
decrement = scan(nonzeros(free.*(1:numel(scan))));
grid(decrement) = grid(decrement) - dec;
grid = max(grid,minval);
grid = min(grid,maxval);

end