function [state] = Control(state,traj,ctrl,boat)
% This function outputs the new desired state of the boat

%--------------------------------------------------------------------------
% Control Algorithm

state = state + [0;0.1;zeros(10,1)];

end