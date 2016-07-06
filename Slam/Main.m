% Integration of RobotX Codes
% This script will allow the boat to be completely autonomous, utilsing
% localisation techniques and path planning methods

clear; close all; clc;
addpath(strcat(pwd,'\Guidance'),strcat(pwd,'\Navigation'),...
strcat(pwd,'\Control'),strcat(pwd,'\Simulation'));

% Initialisaton

Map = struct('Map',[]);
[boat,sen,map,landmarks,goal,pose,obstot] = Info();   

path.time = sen.T;           % Time Interval
path.certain = 15;           % Radial Distance from Boat Certain about Obstalces         
path.qstar = 20;             % Distance Repulsion Function still Valid
path.att = 5.2;              % Damping Ratio for Attraction Gradient
path.rep = 120;              % Damping Ratio for Repulsion Gradient
path.bar = 0.4;              % Damping Ratio for Barrier Gradient
path.rad = 1.2;              % Radius from Obstacle to Ensure No Collision
path.err = 1.5;              % Target Error from Goal  
path.iterations = 5000;      % Maximum Number of Iterations Before Exiting


fig.boatline = 1.3;                % Boat Line Thickness
fig.boatcolour = [0 0 1];          % Boat Colour
fig.radcolour = [0 1 1];           % Sensor Border Colour
fig.obscolour = [1 0 0];           % Colour of Obstacles 
fig.lumpcolour = [0 1 0];          % Colour of Lumping of Obstacles
fig.linwid = 2;                    % Line Width
fig.offset = 0.02;                 % Distance Gate Names is From Plot
fig.start = (1:3)';                % Starting Gates
fig.finish = char(88:90)';         % End Gates        
fig.pause = 0.01;                  % Pause Time During Animation
fig.chars = (1:3)';               % Position of Fonts

displace = abs(pose(1:2) - goal);
gridmap = repmat(Map,1,path.iterations);
gridmap(1).Map = zeros(round(map.length/map.res)+1,round(map.width/map.res)+1); 
pdiag = zeros(length(pose),path.iterations);
loop = 1;

% Plotting Purposes Only
ang = zeros(1,path.iterations);
dis = zeros(2,path.iterations);
vel = zeros(2,path.iterations);
acc = zeros(2,path.iterations);
dis(:,1) = [pose(1) pose(2)]';
animatetime = 0;
tic

% Simulation

init = 1;
times = zeros(1,path.iterations);
while (norm(displace) > path.err) && (loop < path.iterations) 
    % Obtain Sensor Data (GPS,IMU,Camera)
    gps = pose(1:3);  
    imu = pose(4:6); 
    if init
        est = [gps;imu;zeros(6,1)];
        init = 0;
    end

    [rcQC,thQC,camob,radii,rcLC,thLC,camlnd,lndid] = CameraData(pose,sen,obstot,boat.height,landmarks);   %Done! (Get Rid of Cats)
    [gridmap(loop).Map,yob,sen.yobhat,sen.oblen] = UpdateMap(gridmap(loop).Map,[gps;imu],rcQC,thQC,radii,camob,sen,map,boat.height); % Need to Do ObMeasurements
    
    
    
    [yland,sen.ylandhat,sen.landlen] = LandMeasurements(est,gps,imu,rcLC,thLC,camlnd,lndid,landmarks,sen); % Do This?
    
    
    sensordata = [gps;imu;yob;yland];
    [est,sen.P] = UKF(sen.P,est,[],sensordata,@ProcessModel,@MeasurementModel,sen);                      
    % Obtain New Point in Trajectory with PFM
    [globs,points] = Global(gridmap(loop).Map,est(1:2),boat,path.certain,map);                    
    desired = PathPlanning(goal,globs,est,path,map.width,boat);
    % Control the Torques of the Boat to move to Desired Location
    [locxy,locpsi,locvxy,locvpsi,locacc] = Control(desired);
    % Update and Repeat
    pose = [locxy;est(3:5);locpsi;locvxy;est(9:11);locvpsi];
    displace = abs(pose(1:2) - goal);
    pdiag(:,loop) = diag(sen.P.'*sen.P);
    loop = loop + 1; 
    gridmap(loop).Map = gridmap(loop-1).Map; 
   
    
    % Animation Purposes Only
    dis(:,loop) = pose(1:2,1);
    ang(:,loop) = pose(6,1);
    vel(:,loop) = pose(7:8,1);
    acc(:,loop) = locacc;
    animatetoc = Animate(fig,landmarks,obstot,gridmap(loop).Map,boat,map,dis,ang,goal,loop,pdiag,path);
    drawnow();%,points);
    animatetime = animatetime + animatetoc;
end

%% Plotting and Exiting Script
  
timer = toc;
time = timer - animatetime;
disall = sqrt((dis(1,1:loop)-dis(1,1)).^2 + (dis(2,1:loop)-dis(2,1)).^2);
velall = sqrt(vel(1,1:loop).^2 + vel(2,1:loop).^2);
accall = sqrt(acc(1,1:loop).^2 + acc(2,1:loop).^2);
figure(2)
subplot(3,1,1)
plot(1:loop,disall)
title('Distance(m)')
axis([0 loop 0 max(disall)]);
subplot(3,1,2)
plot(1:loop,velall)
title('Velocity(m/s)')
hold on
plot(1:loop,boat.vmax*ones(loop),'r-',1:loop,-boat.vmax*ones(loop),'r-');
axis tight;
subplot(3,1,3)
plot(1:loop,accall)
title('Acceleration(m/s/s)')
hold on
plot(1:loop,boat.amax*ones(loop),'r-',1:loop,-boat.amax*ones(loop),'r-');
axis tight;
format short
if loop ~= 1   
    fprintf('Time to Complete Course using PFM: %d\n',time);
end
disp('Program Terminated');
% obstot =  [10,14,14;5,5,5;-0.4699,-0.4699,-0.4699];     % For Testing