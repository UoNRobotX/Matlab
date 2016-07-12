% RobotX GNC 2016

%--------------------------------------------------------------------------
% Initialisaton

clearvars; close all; clc;
addpath(strcat(pwd,'\Guidance'),strcat(pwd,'\Navigation'),...
strcat(pwd,'\Control'),strcat(pwd,'\Simulation'));
[boat,ctrl,path,sen,map,lmrks,obs,goal,state] = Params([1,1]); 
statestd = sqrt([sen.gnoise,sen.inoise,sen.vnoise])';
gridsize = [round([map.length,map.width]./map.res)+1,path.iterations];
gridhist = zeros(gridsize(1),gridsize(2),gridsize(3)); 
phist = cat(2,diag(sen.P),zeros(length(state),path.iterations-1)); 
esthist = zeros(3,path.iterations);
grid = gridhist(:,:,1); count = 1;
displace = abs(state(1:2)-goal); tic;

%--------------------------------------------------------------------------
% Simulation

while (norm(displace) > path.err)&&(count ~= path.iterations)
    est = state + statestd.*randn(12,1);
    [QC,LC,rad,ocam,lcam,lndid] = Camera(state,sen,boat,obs,lmrks);
    [grid,y,yhat] = Nav(grid,est,QC,LC,ocam,lcam,rad,lmrks(:,lndid),...
    sen,boat.height,map);       
    [est,sen.P] = UKF(est,[est(1:6);y],yhat,[],@Process,@Measure,sen);
    [traj] = Path(est,grid,map,path,boat);
    [state] = Control(state,traj,ctrl,boat); 
    displace = abs(state(1:2) - goal);        
    gridhist(:,:,count) = grid;
    esthist(:,count) = est([1,2,6]);
    stahist(:,count) = state([1,2,6]);
    phist(:,count+1) = diag(sen.P.'*sen.P);
    count = count + 1;
end

%--------------------------------------------------------------------------
% Plot Results

Results(gridhist,phist,obs,lmrks,esthist,stahist,goal,boat,sen,map,count);