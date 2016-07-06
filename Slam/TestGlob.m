function [newObs, points] = ProbOccup(obsGrid,thresh,boatWidth,boatPos,plotFlag)
%Takes a POG and output the obstacle points and the dist from boat to Obs

% Parameters
res = map.res;
int = map.int;
bleng = boat.length;
bwidth = boat.width;
lumpgap = sqrt(bleng^2+bwidth^2); 
obscount = 0;
obsmap = zeros(size(gridmap,1),size(gridmap,2));

% Position of Boat in Grid
Posx = (est(1)/res)+1;
Posy = (est(2)/res)+1;
Rad = round(radius/res);

% Limit Cells to be Searched
Limit = round([Posx-Rad,Posx+Rad,Posy-Rad,Posy+Rad]);
for count = 1:length(Limit)
    if Limit(count) < 1
        Limit(count) = 1;
    end   
    if Limit(count) >= size(gridmap,ceil(count/2))
        Limit(count) = size(gridmap,ceil(count/2));
    end
end 

% Find Cells in GridMap Correspinding to each Obstacle.
cells = [];
for count = Limit(1):Limit(2)
    for loop = Limit(3):Limit(4)
        if (gridmap(count,loop) >= thres)&&(obsmap(count,loop) == 0)
            obscount = obscount + 1;
            [obsmap,cells] = FloodFind(obsmap,obscount,gridmap,thres,count,loop,cells,res); 
        end
    end
end

% Place Cells Into Structure for Manipulation
obstemp(1:obscount) =  struct('x',[],'y',[]);
for count = 1:size(cells,2)  
    obstemp(cells(3,count)).x = cat(2,obstemp(cells(3,count)).x,cells(1,count));
    obstemp(cells(3,count)).y = cat(2,obstemp(cells(3,count)).y,cells(2,count));
end
% Ensure Max Spacing of Each Point is Less Than Int
obspoint(1:obscount) = struct('x',[],'y',[]);
for count = 1:obscount
    K = convhull(obstemp(count).x,obstemp(count).y);
    obspoint(count).x = obstemp(count).x(K);
    obspoint(count).x = obstemp(count).y(K);
    for loop = 1:(obstemp(count).x(K) - 1)
        X = [obspoint(count).x(loop);obspoint(count).x(loop+1)];
        Y = [obspoint(count).y(loop);obspoint(count).y(loop+1)];
        if norm([X(1),Y(1)],[X(2),Y(2)]) > int
           % Vertical Line
           if(X(1) == X(2))
               Yint = min(Y):int:max(Y);
               Xint = X(1)*ones(length(Yint),1);
           % Horizontal Line
           elseif (Y(1) == Y(2))
               Xint = min(X):int:max(X);
               Yint = Y(1)*ones(length(Xint),1);
           % Sloped Line
           else
               M =(Y(2)-Y(1))/(X(2)-X(1));
               B = Y(1)-(M*X(1));
               Xint = min(X):int:max(X);
               Yint = M*Xint + B;
           end
           obspoint(count).x = cat(2,obspoint(count).x,Xint);
           obspoint(count).y = cat(2,obspoint(count).y,Yint);
        end
    end
end

% Determine which Obstacles are in Close Proximity to One Another
Lump = [];
for count = 1:obscount
    X = obspoint(count).x;
    Y = obspoint(count).y;
    Current = [X,Y];
    for loop = count+1:obscount
        Xev = obspoint(loop).x;
        Yev = obspoint(loop).y;
        Eval = [Xev,Yev];
        Dist = pdist2(Current,Eval);
        if min(Dist) <= lumpgap
            Lump = cat(2,Lump,[count;loop]);
        end
    end
end
       
%% HERE!

% Lump Obstacles
if ~isempty(Lump)
    lumpobs(1:obscount) = struct('lump',0);
    for count = 1:size(Lump,2)
        flag = 0;
        Top = min(Lump(:,count));
        Bottom = max(Lump(:,count));
        for loop = 1:obscount
            for index = 1:length(lumpobs(loop).lump)
                if lumpobs(loop).lumpp(index) == Top
                    flag = 1;
                    if lumpobs(loop).lumpp(index) == Bottom
                        flag = 2;
                    end
                elseif lumpobs(loop).lumpp(index) == Bottom
                    flag = 3;
                end
            end
        end
    end
    
    
else
%     Ignore = 0;
%     for count = 1:size(Lump,2)
%         flag = [];
%         secflag = [];
%         for loop = 1:size(Ignore,2)
%             % Already been Lumped with Another Obstalce
%             if (Lump(1,count) == Ignore(loop))
%                 flag = cat(2,flag,loop);        
%             end
%             if (Lump(2,count) == Ignore(loop))
%                 secflag = cat(2,secflag,loop);        
%             end
%         end
%         if ~isempty(flag)
%             obspoint(count).x = cat(2,obspoint(Lump(1,count)).x,obspoint(Lump(2,count).x));
%             obspoint(count).y = cat(2,obspoint(Lump(1,count)).y,obspoint(Lump(2,count).y));
%         else
%  
%             
%         end
%         Ignore = cat(2,Ignore,Lump(2,count));
%     end
    % Intervals Again!!! and Convhull Again!!!
end


    

    while 1
        for i = 1:obsCount;       %rows of matrix
            for j = i+1:obsCount; %column of matrix, i+1 means always skips the (i,j) term of matrix
                minDist = minDistMatrix(i,j);
                %if the obstacles are too close together
                if minDist < boatWidth
                    
                    %add obstacle 2's points to obstacle 1
                    x = [points(i).x;points(j).x];
                    y = [points(i).y;points(j).y];
                    
                    % copy values into points
                    points(i).x = x;
                    points(i).y = y;
                    
                    %delete 2nd points
                    points(j) = [];
                    obsCount = obsCount -1;
                    %restart the whole process
                    obsGroupFlag =1;
                    breakFlag = 1;


% Calculate the distance from the boat to nearest obstacle

closestObsPoint = zeros(2,obsCount);
minDistBoat = zeros(obsCount,1);
for i = 1:obsCount;
    
    x = points(i).x;
    y = points(i).y;
    
    
    xnext = boatPos(1);
    ynext = boatPos(2);
    
    %calculate the distance to all the points
    now = [x,y];
    next = [xnext,ynext];
    
    %calculate the distance between each point in now to each in next
    distance = pdist2(now,next);
    
    %find the minimum distance between the two points
    [minDist,index] = min(distance(:));
    [locNow , locNext] = ind2sub(size(distance),index);  % min at locNow row of Now and locNext row of next
    clpoint = [x(locNow),y(locNow)]';                  % closest point in the form x on top row y on bottom
    closestObsPoint(:,i) = clpoint;
    
    minDistBoat(i) = minDist;
end
clPoints = closestObsPoint;

end

points = GetConvPoints(points,0);
newObs = [clPoints(1,:);clPoints(2,:) ; minDistBoat'];
end