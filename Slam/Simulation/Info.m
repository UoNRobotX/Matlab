function [boat,sen,map,landmarks,goal,pose,obstacles] = Info()
%This function collects all neccessary data from user to begin algorithm

[boat,sen,map] = Params();

start = [5,20,35];
pose = [0,start(1),boat.height,zeros(1,9)]';
goal = [map.length,start(1)]';
landx = [zeros(1,4),map.length.*ones(1,4)];
landy = repmat((map.width/3)*[0,1,2,3],1,2);
landz = -map.radius*ones(1,8);
landmarks = [landx;landy;landz];
obsx = repmat((map.length/4).*[1,2,3],1,3);
obsy = repmat((map.width/4).*[1,2,3],1,3); index = sort(repmat((1:3),1,3));
obsz = sen.radii([1,1,1,2,2,2,3,3,3]);
obstacles = [obsx;obsy(index);-obsz];

end