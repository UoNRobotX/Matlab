function [y,yhat] = ObsVect(gridy,grid,map,rnBN,scan,range)
% This function returns vectors of distances between the boat 
% and the seen obstalces(y),as well as the distances between the 
% boat and the stored obstalces in the virtual grid (yhat)

res = map.res;
measure = range.*ones(360,2);
allthres = [map.inc,map.thres];
allgrid = cat(3,gridy,grid);

%--------------------------------------------------------------------------
% Obtain Data

for count = 1:2 
    thres = allthres(count); 
    current = allgrid(:,:,count); 
    occ = scan(current(scan) >= thres);
    [occn,occe] = ind2sub(size(grid),occ); 
    occupied = [occn;occe];
    vect = res.*(occupied -0.5) - repmat(rnBN,1,size(occupied,2)); 
    norms = sqrt(sum(vect.^2));  
    angle = round(atan2(vect(2,:),vect(1,:))*(180/pi)); 
    angle(angle <= 0 ) = angle(angle <= 0 ) + 360; 
    angle(angle > 360) = angle(angle > 360) - 360;
    for loop = 1:size(angle,2)
        if (norms(loop) < measure(angle(loop),count))
            measure(angle(loop),count) = norms(loop);
        end
    end
end

%--------------------------------------------------------------------------
% Form Outputs

y = measure(:,1);  
yhat = measure(:,2);

end