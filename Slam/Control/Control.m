function [realxy,realpsi,realvxy,realvpsi,realacc] = Control(desired)
%This function outputs the new real two dimensional states of the boat
%using control optimisation techniques

% Testing Control System
realxy = desired(1:2);
realpsi = desired(3);
realvxy = desired(4:5);
realvpsi = desired(6);
realacc = desired(7:8);

% Real Control System
% STEPH HERE!

end