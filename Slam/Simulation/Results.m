function [] = Results(grid,est,start,goal,boat,sen,map,lim,err,ctime,mtime)
% This function plots the results to the user

%--------------------------------------------------------------------------
% Parameters

pose = est(1:2,:);
triangle = [-1,1,0,-1;-1,-1,1,-1];
dim = repmat([boat.length,boat.width]',1,4)./2;
angle = reshape(est(3,:)-pi/2,1,1,size(est,2));
rot = [cos(angle),-sin(angle);sin(angle),cos(angle)];

%--------------------------------------------------------------------------
% Output Results

figure(1); clf;
for ind = 1:lim
    tri = (rot(:,:,ind)*triangle).*dim + repmat(pose(:,ind),1,4);
    subplot(7,11,[1:6,23:28]); hold on;
    imagesc([0,map.length],[0,map.height],grid(:,:,ind,1));
    scatter(start(2),start(1),70,'g','filled');
    scatter(goal(2),goal(1),70,'r','filled');
    plot(tri(2,:),tri(1,:),'Color','c','LineWidth',1); hold off;
    axis([0,map.length,0,map.height]); title('Matlab Stored Map');
    subplot(7,11,[45:50,67:72]); hold on  
    imagesc([0,map.length],[0,map.height],grid(:,:,ind,2));
    scatter(start(2),start(1),70,'g','filled');
    scatter(goal(2),goal(1),70,'r','filled');
    plot(tri(2,:),tri(1,:),'Color','c','LineWidth',1); hold off;
    axis([0,map.length,0,map.height]); title('C++ Stored Map');
    subplot(7,11,[8:11,30:33]);
    plot(1:ind,ctime(1:ind),'k'); hold on;
    plot(1:ind,mtime(1:ind),'g'); hold off;
    title('Execution Time [s]'); legend('C++','Matlab');
    subplot(7,11,[52:55,74:77]);
    plot(1:ind,err(:,1:ind)); title('Maximum Vector Error [m]');
    legend('Y','Y_{hat}');
    pause(sen.time);
end
end