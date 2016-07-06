function [boat,sen,map] = Params()
%This function has all parameters for each component of the system

%--------------------------------------------------------------------------
% Boat Parameters

boat.amax = 10;             % Max accelaration [m/s/s]
boat.vmax = 4;              % Max velcoity [m/s]
boat.omega = 3;             % Max angular velocity [rad/s]
boat.length = 3.5;          % Length of boat [m]
boat.width = 2.5;           % Width of boat [m]
boat.height = -1.25;        % Height of boat [m] [F R D]

%--------------------------------------------------------------------------
% Map Parameters

map.res = 0.2;                          % Resolution of Map
map.int = 0.1;                          % Maximum Interval Between Points on Lumped Obstaclesde
map.dec = 4;                            % Decrement Value
map.inc = 0.5;                          % Increment Value
map.max = 50;                           % Max Grid Value
map.min = -50;                          % Min Grid Value
map.thres = 0.5;                        % Occupied Threshold
map.width = 40;                         % Width of Field
map.length = 60;                        % Length of Field
map.radius = 0.2286;                    % Radius of Landmarks

%--------------------------------------------------------------------------
% Orientation of Cameras on Boat

rbCB1 = [0.28,0,-0.1]';                 % Camera 1 to Cross on Boat
rbCB2 = [0,-0.4,-0.1]';                 % Camera 2 to Cross on Boat
rbCB3 = [0,0.4,-0.1]';                  % Camera 3 to Cross on Boat
rbCB4 = [-1.21,0,-0.1]';                % Camera 4 to Cross on Boat
RbCB1 = [0 0 1; 1 0 0; 0 1 0];          % Camera 1 Rotation to Boat Coord
RbCB2 = [-1 0 0; 0 0 1; 0 1 0];         % Camera 2 Rotation to Boat Coord
RbCB3 = [1 0 0; 0 0 -1; 0 1 0];         % Camera 3 Rotation to Boat Coord
RbCB4 = [0 0 -1; -1 0 0; 0 1 0];        % Camera 4 Rotation to Boat Coord

%--------------------------------------------------------------------------
% Sensor Parameters

sen.T = 0.2;                         % Time interval
sen.cameras = 4;
sen.rad = 45;                          % Radial Distance of cameras [m]
sen.deg = deg2rad([191;40;40;191]);     % View of Cameras [rad]
sen.rnoise = 1.5;                       % Noise of Range to Bouy [m]
sen.bnoise = deg2rad(5);                % Noise of Bearing to Bouy [rad]
sen.gpsnoise = 1.2;                     % GPS Noise [m]   
sen.imunoise = deg2rad(5);              % IMU Noise [rad]
sen.gridnoise = map.res/sqrt(12);       % Grid Noise [m]
sen.radii = 1.8.*[0.216,0.343,0.4699];  % Possible Radii of Obstacles
sen.rbCB = [rbCB1,rbCB2,rbCB3,rbCB4];   % All Trans of Cameras to Boat
sen.RbCB = [RbCB1,RbCB2,RbCB3,RbCB4];   % All Rotations of Cameras to Boat
Qned = 1e-12.*[1,1,1];                  % Model Certainty for NED 
Qang = 1e-17.*deg2rad([1,1,1]);         % Model Certainty for Eul Ang
Qneddot = 1e-16.*[1,1,1];               % Model Certainty for NED Deriv
Qangdot = 1e-16.*deg2rad([1,1,1]);      % Model Certainty for Eul Ang Deriv
sen.Q = [Qned,Qang,Qneddot,Qangdot];    % Total Model Certainty
sen.P = 10e-2.*diag(ones(12,1));        % Inital Covariance Estimate

%--------------------------------------------------------------------------

end