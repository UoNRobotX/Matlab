% RobotX GNC 2016

%--------------------------------------------------------------------------
% Initialisaton

clearvars; close all; clc;
addpath(strcat(pwd,'\Guidance'),strcat(pwd,'\Estimation'),...
strcat(pwd,'\Mapping'),strcat(pwd,'\Control'),strcat(pwd,'\Simulation'));
[boat,ctrl,path,sen,map,lmrks,obs,goal,state] = Params([1,1]); 
gridsize = [round([map.height,map.length]./map.res)+1,path.iterations];
gridhist = zeros(gridsize(1),gridsize(2),gridsize(3),2); 
start = state(1:2); esthist = zeros(3,path.iterations);
gridm = gridhist(:,:,1); gridc = gridm; err = zeros(2,path.iterations);
ctime = zeros(path.iterations); mtime = ctime;
displace = abs(state(1:2)-goal); count = 1;

%--------------------------------------------------------------------------
% Simulation

while (norm(displace) > path.err)&&(count ~= path.iterations)
    
    % Obtain Camera Data
    est = state;
    [obsinfo,lndinfo,obscam,lndcam] = Camera(state,sen,obs,lmrks);
    
    % C++ - Matlab Testing   
    [gridm,ym,ymhat,gridc,yc,ychat,ctoc,mtoc] = Update(gridm,gridc,est,...
    obsinfo,lndinfo,obscam,lndcam,lmrks,sen,map); 
    gridhist(:,:,count,1) = gridm; gridhist(:,:,count,2) = gridc;
    err(:,count) = [max(abs(ym-yc)), max(abs(ymhat-ychat))]';
    ctime(count) = ctoc; mtime(count) = mtoc;
    
    % Move Vehicle and Store Elements
    [traj] = Path(est,gridm,map,path,boat);
    [state] = Control(state,traj,ctrl,boat); 
    displace = abs(state(1:2) - goal);        
    esthist(:,count) = est([1,2,6]);
    count = count + 1;
end

%--------------------------------------------------------------------------
% Plot Results

Results(gridhist,esthist,start,goal,boat,sen,map,count-1,err,ctime,mtime);