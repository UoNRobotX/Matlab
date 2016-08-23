function [grid,gridy] = Grid(grid,rnCN,RnCN,QC,cam,rad,scan,cells,map)
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
     
current = grid;  % Stores a copy of grid before changes are made
grid(scan) = max(grid(scan)-dec,minval);  % Decriments all cells within the radius (lower bounded by minval)
rnGN = res.*[cells-0.5;zeros(1,size(cells,2))]; len = size(rnGN,2);
for count = 1:obs
    rnPNmat = repmat(rnPN(:,count),1,len);
    rcQCmat = repmat(rcQC(:,count),1,len);
    rnCNmat = repmat(rnCN(:,cam(count)),1,len);
    rcGC = (RnCN(:,cind(:,count))')*(rnGN - rnCNmat);
    magnitude = sqrt(sum((rnPNmat-rnGN).^2));   
    ind = nonzeros((magnitude <= rad(count)).*(1:len));
    circle = sub2ind(size(grid),cells(1,ind),cells(2,ind)); % Obtains a vector of linear indicies for all grid cells within obstacle radius
    if ~isempty(circle)
        deccir = repmat(dec,1,length(circle));
        change = (current(circle) - grid(circle)) ~= dec;
        indcir = nonzeros(change.*(1:length(circle))); 
        if ~isempty(indcir)
            deccir(indcir) = 0;
        end
        grid(circle) = min(grid(circle) + inc + deccir,maxval); % Increments all cells within radius that are empty (upper bounded by maxval)
    end
    norms = sqrt(sum(rcGC.^2));
    dotprod = sum(rcQCmat.*rcGC)./norms; thGC = acos(dotprod);   
    bound = (thGC <= thQC(count)).*(dotprod >= 0);  
    ind = nonzeros(bound.*(1:len));    
    allellipse = sub2ind(size(grid),cells(1,ind),cells(2,ind));
    ellipse = allellipse(~ismember(allellipse,circle));
    if ~isempty(ellipse)
        decell = repmat(dec,1,length(ellipse));
        change = (current(ellipse) - grid(ellipse)) ~= dec;
        indell = nonzeros(change.*(1:length(ellipse)));    
        if ~isempty(indell)
            decell(indell) = 0;
        end
        grid(ellipse) = max(grid(ellipse) + decell,minval); 
    end 
end 
gridy = grid - current; % This is incorrect. Need to modify gridy within the loop

end