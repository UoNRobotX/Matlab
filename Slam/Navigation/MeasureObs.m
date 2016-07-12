function [y,yhat] = MeasureObs(gridy,grid,map,rnBN,scan)
% This function returns vectors of inverse distances between the boat 
% and the seen obstalces(y),as well as the inverse distances between the 
% boat and the stored obstalces in the virtual grid (yhat)

res = map.res;
measure = zeros(360,2);
allthres = [map.inc,map.thres];
allgrid = cat(3,gridy,grid);

%--------------------------------------------------------------------------
% Obtain Data

for count = 1:2
    thres = allthres(count);
    current = allgrid(:,:,count);
    occ = scan(current(scan) >= thres);
    [occx,occy] = ind2sub(size(grid),occ);
    occupied = [occx;occy];
    vect = res.*(occupied - repmat(rnBN,1,size(occupied,2)) - 0.5);
    norms = sqrt(sum(vect.^2));  
    [sortnorm,ind] = sort(norms);
    sortvect = vect(:,ind);
    angle = round(atan2(sortvect(2,:),sortvect(1,:))*(180/pi));
    angle(angle <= 0 ) = angle(angle <= 0 ) + 360;
    angle(angle > 360) = angle(angle > 360) - 360;
    [beamind,normind] = unique(angle);
    measure(beamind,count) = sortnorm(normind);    
end

%--------------------------------------------------------------------------
% Form Outputs

y = measure(:,1); 
yhat = measure(:,2);

end