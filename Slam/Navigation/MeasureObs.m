function [y,yhat] = MeasureObs(gridy,grid,map,rnBN,scan)
% This function returns vectors of distances between the boat 
% and the seen obstalces(y),as well as the distances between the 
% boat and the stored obstalces in the virtual grid (yhat)

res = map.res;
measure = zeros(360,2);
allthres = [map.inc,map.thres];
allgrid = cat(3,gridy,grid);

%--------------------------------------------------------------------------
% Obtain Data

for count = 1:2  % First loop: For changes to grid, Second loop: For actual grid
    thres = allthres(count); % 1: thres = inc. 2: thres = occupied threshold
    current = allgrid(:,:,count); % Extracts the grid we are using
    occ = scan(current(scan) >= thres); % Extracts linear indices within the observation radius that are believed to be occupied
    [occx,occy] = ind2sub(size(grid),occ); % Converts the linear index list back into [m,n] indicies
    occupied = [occx;occy];
    vect = res.*(occupied - repmat(rnBN,1,size(occupied,2)) - 0.5); % Get the real world vector between the boat position and occupied cells
    norms = sqrt(sum(vect.^2));  % Gets the magnitude of the vectors
    [sortnorm,ind] = sort(norms); % Sorts these magnitudes
    sortvect = vect(:,ind); % Uses sorted magnitudes to sort their associated vectors
    angle = round(atan2(sortvect(2,:),sortvect(1,:))*(180/pi)); % Calculate bearing angle to obstacle
    angle(angle <= 0 ) = angle(angle <= 0 ) + 360; % Bounds angle between [1:360]
    angle(angle > 360) = angle(angle > 360) - 360;
    [beamind,normind] = unique(angle); % Gets rid of repeated angles
    measure(beamind,count) = sortnorm(normind); % Removed the magnitudes associated with the removed angles
end

%--------------------------------------------------------------------------
% Form Outputs

y = measure(:,1);  % Obstacles seen this scan
yhat = measure(:,2); % Obstacles that remain above occupied threshold

end