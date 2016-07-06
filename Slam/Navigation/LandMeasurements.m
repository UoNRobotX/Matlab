function [y,yhat,landlen] = LandMeasurements(est,gps,imu,rcLC,theta,Cam,land,landmarks,sen)
%This function returns the landmark measuremnts and approximate
%measurements. These measuremnts are the bearings and inverse ranges
%between the boat and the landmarks.

% Parameters
Cameras = sen.cameras;

% Finding Y Using Estimated State and Camera Data

Roll = est(4);
Pitch = est(5);
Yaw = est(6);

rnBN = [est(1);est(2);est(3)];
RnBN = [ cos(Yaw)*cos(Pitch) -sin(Yaw)*cos(Roll)+cos(Yaw)*sin(Pitch)*sin(Roll) sin(Yaw)*sin(Roll)+cos(Yaw)*cos(Roll)*sin(Pitch)  ;
         sin(Yaw)*cos(Pitch) cos(Yaw)*cos(Roll)+sin(Roll)*sin(Pitch)*sin(Yaw)  -cos(Yaw)*sin(Roll)+sin(Yaw)*cos(Roll)*sin(Pitch) ;
         -sin(Pitch)         cos(Pitch)*sin(Roll)                              cos(Pitch)*cos(Roll)                             ];

rbCB = sen.rbCB;                
RbCB = sen.RbCB;                
rnCN = zeros(3,Cameras); 
RnCN = zeros(3,3*Cameras);
for count = 1:size(rnCN,2)
    index = (3*count)-2;
    rnCN(:,count) = rnBN+(RnBN*rbCB(:,count));
    RnCN(:,index:index+2) = RnBN*RbCB(:,index:index+2);
end

% Get Global 2D Position of Landmark Centres
invrangey = zeros(1,size(rcLC,2));
bearingy = zeros(1,size(rcLC,2));
for count = 1:size(rcLC,2)
    index = (3*Cam(count))-2;
    rnPN = (RnCN(:,index:index+2)*((-landmarks(3,Cam(count))/sin(theta(count)))*rcLC(:,count)))+rnCN(:,Cam(count));
    bearingy(1,count) = atan2(rnPN(2)-rnBN(2),rnPN(1)-rnBN(1));
    invrangey(1,count) = 1/norm(rnPN(1:2)-rnBN(1:2));
end
y = [bearingy,invrangey]';
landlen = length(y);

% Finding Yhat Using GPS Given Locations

markers = landmarks(:,land);

Theta = imu(1);
Phi = imu(2);
Psi = imu(3);

Rhat = [ cos(Psi)*cos(Theta) -sin(Psi)*cos(Phi)+cos(Psi)*sin(Theta)*sin(Phi) sin(Psi)*sin(Phi)+cos(Psi)*cos(Phi)*sin(Theta);
      sin(Psi)*cos(Theta) cos(Psi)*cos(Phi)+sin(Phi)*sin(Theta)*sin(Psi)  -cos(Psi)*sin(Phi)+sin(Psi)*cos(Phi)*sin(Theta) ;
      -sin(Theta)         cos(Theta)*sin(Phi)                             cos(Theta)*cos(Phi)                             ];
  
invryhat = zeros(1,size(markers,2));
bearyhat = zeros(1,size(markers,2));
for count = 1:size(markers,2)
    Bx = Rhat*(markers(:,count) - [gps(1),gps(2),gps(3)].');
    bearyhat(1,count) = atan2(Bx(2),Bx(1));
    invryhat(1,count) = 1/sqrt(Bx(1)^2+Bx(2)^2);
end
yhat = [bearyhat,invryhat]';

end