function [globs,plotpoints] = Global(gridmap,est,boat,radius,map)
%This function utilses the gridmap to lump obstalces that are close
%together that may form a local minimum or saddle point. After lumping the
%obstalces, the function then returns the position of the closest point on
%each obstalce to the boat, as well as the distance between these points
%and the boat.

% Parameters
res = map.res;
int = map.int;
thres = map.thres;
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
%     if length(obstemp(count).x) > 2
%         K = convhull(obstemp(count).x,obstemp(count).y);
%     else
%         if length(obstemp(count).x) == 2
%             K = [1,2];
%         else
%             K = 1;
%         end
%     end
    obspoint(count).x = obstemp(count).x;
    obspoint(count).y = obstemp(count).y;
    for loop = 1:(length(obspoint(count).x) - 1)
        X = [obspoint(count).x(loop);obspoint(count).x(loop+1)];
        Y = [obspoint(count).y(loop);obspoint(count).y(loop+1)];
        if sqrt((X(1)-X(2))^2+(Y(1)-Y(2))^2) > int
           % Vertical Line
           if(X(1) == X(2))
               Yint = min(Y):int:max(Y);
               Xint = X(1)*ones(1,length(Yint));
           % Horizontal Line
           elseif (Y(1) == Y(2))
               Xint = min(X):int:max(X);
               Yint = Y(1)*ones(1,length(Xint));
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
    for loop = count+1:obscount
        Xev = obspoint(loop).x;
        Yev = obspoint(loop).y;
        Dist = zeros(length(X),length(Xev));
        for index = 1:length(X)
            for key = 1:length(Xev)
                Dist(index,key) = sqrt((X(index)-Xev(key))^2+(Y(index)-Yev(key))^2);
            end
        end
        minvect = min(Dist);
        if min(minvect) <= lumpgap
            Lump = cat(2,Lump,[count;loop]);
        end
    end
end
       
% Lump Obstacles if Needed
if ~isempty(Lump)
    for count = 1:size(Lump,2)
        Top = min(Lump(:,count));
        Bottom = max(Lump(:,count));
        if count ~= 1
            for loop = 1:count-1
                % Used Already
                if max(Lump(:,loop)) == Top
                    Top = min(Lump(:,loop));
                    break;
                end
            end
            for index = 1:count-1
                if (min(Lump(:,index)) == Top)&&(max(Lump(:,count)) == Bottom)
                    Bottom = 0;
                end
            end
        end    
        if Bottom ~= 0
            obspoint(Top).x = cat(2,obspoint(Top).x,obspoint(Bottom).x);
            obspoint(Top).y = cat(2,obspoint(Top).y,obspoint(Bottom).y);
            obspoint(Bottom).x = [];
            obspoint(Bottom).y = [];
        end
    end
    % Make Better Points Along Boundary of Lumped Obstacles
    for count = 1:obscount
        if ~isempty(obspoint(count).x)
%            if length(obspoint(count).x) > 2
%                 K = convhull(obspoint(count).x,obspoint(count).y);
%            else
%                 if length(obstemp(count).x) == 2
%                     K = [1,2];
%                 else
%                     K = 1;
%                 end
%            end
%            obspoint(count).x = obspoint(count).x(K);
%            obspoint(count).y = obspoint(count).y(K);
           for loop = 1:(length(length(obspoint(count).x)) - 1)
               X = [obspoint(count).x(loop);obspoint(count).x(loop+1)];
               Y = [obspoint(count).y(loop);obspoint(count).y(loop+1)];
               if sqrt((X(1)-X(2))^2+(Y(1)-Y(2))^2) > int
                  % Vertical Line
                  if(X(1) == X(2))
                      Yint = min(Y):int:max(Y);
                      Xint = X(1)*ones(1,length(Yint));
                  % Horizontal Line
                  elseif (Y(1) == Y(2))
                      Xint = min(X):int:max(X);
                      Yint = Y(1)*ones(1,length(Xint));
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
    end
end
  
% Calculate the Distance from the Boat to Nearest Obstacle and Store the
% Point on the Obstacle

ClosetPoint = zeros(2,obscount);
MinDist = zeros(1,obscount);
index = [];
key = 0;
for count = 1:obscount
    if ~isempty(obspoint(count).x)
        X = obspoint(count).x;
        Y = obspoint(count).y;
        Dist = zeros(length(X),1);
        for loop = 1:length(X)
            Dist(loop,1) = sqrt((X(loop)-est(1))^2+(Y(loop)-est(2))^2);
        end
        key = key + 1;
        [MinDist(1,key),row] = min(Dist);
        ClosetPoint(:,count) = [X(row);Y(row)];
        index = cat(2,index,count);
    end
end
ObPoints = ClosetPoint(:,index);
globs = [ObPoints;MinDist(1,1:key)];

% For Plotting Only
plotpoints = [];
for count = 1:length(index)
   obstacle = [obspoint(count).x;obspoint(count).y];
   plotpoints = cat(2,plotpoints,obstacle); 
end
end