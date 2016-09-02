function [boat,ctrl,path,sen,map,land,obs,goal,state] = Params(ind)
% This function collects all neccessary data to begin simulation
% MODIFY [Q,R] VALUES FOR UKF

%--------------------------------------------------------------------------
% Boat Parameters

boat.constraints = [];                  % Physical Constraints of Boat
boat.length = 3.5;                      % Length of boat [m]
boat.width = 2.5;                       % Width of boat [m]
boat.height = -1.25;                    % Height of boat [m]

%--------------------------------------------------------------------------
% Control Parameters

ctrl = [];                              % Control Structure

%--------------------------------------------------------------------------
% Path Parameters

path = [];                              % Path Structure
path.err = 2.5;                         % Distance Error to Goal [m]
path.iterations = 1000;                 % Max Iterations before Stopping

%--------------------------------------------------------------------------
% Map Parameters

map.dec = 0.5;                          % Decrement Value
map.inc = 4;                            % Increment Value
map.max = 50;                           % Max Occupancy Grid Value
map.min = -50;                          % Min Occupancy Grid Value
map.thres = 20;                         % Occupied Threshold
map.res = 0.2;                          % Resolution of Map [m]
map.width = 40;                         % Width of Field [m]
map.length = 60;                        % Length of Field [m]
map.lrad = 0.2286;                      % Radii of Landmarks [m]
map.orad = 1.8.*[0.216,0.343,0.4699];   % Radii of Obstalces [m]

%--------------------------------------------------------------------------
% Camera Vectors and Rotation Matrices

rbCB1 = [0.28,0,-0.1]';                 % Camera 1 [FRD] to Cross [m]
rbCB2 = [0,-0.4,-0.1]';                 % Camera 2 [FRD] to Cross [m]
rbCB3 = [0,0.4,-0.1]';                  % Camera 3 [FRD] to Cross [m]
rbCB4 = [-1.21,0,-0.1]';                % Camera 4 [FRD] to Cross [m]
RbCB1 = [0 0 1; 1 0 0; 0 1 0];          % Camera 1 Rotation
RbCB2 = [-1 0 0; 0 0 1; 0 1 0];         % Camera 2 Rotation 
RbCB3 = [1 0 0; 0 0 -1; 0 1 0];         % Camera 3 Rotation 
RbCB4 = [0 0 -1; -1 0 0; 0 1 0];        % Camera 4 Rotation

%--------------------------------------------------------------------------
% Process Model Uncertainty

Qned = 1e-12.*[1,1,1];                  % NED [m^2]
Qang = 1e-17*(pi/180).*ones(1,3);       % Roll Pitch Yaw [rad^2]
Quvw = 1e-16.*[1,1,1];                  % Position Velocities [m^2/s^2]
Qpqr = 1e-16*(pi/180).*ones(1,3);       % Angular Velocities [rad^2/s^2]

%--------------------------------------------------------------------------
% Sensor Parameters

sen.camera = 4;                         % Number of Cameras
sen.time = 0.2;                         % Sample Time [s]
sen.rad = 45;                           % Radial Distance of cameras [m]
sen.fov = (pi/180).*[191,40,40,191];    % Camera Field of View [rad]
sen.cnoise = 1e-5.*[1,0.1*(pi/180)];    % Camera Noise [m^2,rad^2]  
sen.gnoise = 1e-5.*[10,10,10];          % GPS Noise [m^2];
sen.inoise = 1e-5*(pi/180).*[10,1,1];   % IMU Noise [rad^2]; 
sen.vnoise = 1e-5.*ones(1,6);           % Deriv Noise [m^2/s^2,rad^2/s^2]
sen.mnoise = map.res/sqrt(12);          % Grid Resolution Noise [m^2]
sen.Q = [Qned,Qang,Quvw,Qpqr];          % Model Certainty
sen.P = 10e-1.*diag(ones(12,1));        % Inital Covariance Estimate
sen.rbCB = [rbCB1,rbCB2,rbCB3,rbCB4];   % Camera Translations on Boat [m]
sen.RbCB = [RbCB1,RbCB2,RbCB3,RbCB4];   % Camera Rotations on Boat

%--------------------------------------------------------------------------
% Landmark & Obstalce Positions

start = [5,20,35];
state = [0,start(ind(1)),boat.height,zeros(1,9)]';
goal = [map.length,start(ind(2))]';
landx = [zeros(1,4),map.length.*ones(1,4)];
landy = repmat((map.width/3)*[0,1,2,3],1,2);
landz = -map.lrad*ones(1,8);
land = [landx;landy;-landz];
obsx = repmat((map.length/4).*[1,2,3],1,3);
obsy = repmat((map.width/4).*[1,2,3],1,3); 
index = sort(repmat((1:3),1,3));
obsz = map.orad([1,1,1,2,2,2,3,3,3]);
obs = [obsx;obsy(index);-obsz];

end