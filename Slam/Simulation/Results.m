function [] = Results(grid,pdiag,obs,lmks,est,goal,boat,sen,map,count,time)
% This function plots the results to the user

%--------------------------------------------------------------------------
% Parameters

ind = 0:0.01:2*pi;
triangle = [-1,1,0,-1;-1,-1,1,-1];
curpos = []; currad = []; pose = est(1:2,:);
dim = repmat([boat.length,boat.width]',1,4)./2;
angle = reshape(est(3,:)-pi/2,1,1,size(est,2));
rot = [cos(angle),-sin(angle);sin(angle),cos(angle)];
sine = reshape(sin(ind),numel(ind),1);
cosine = reshape(cos(ind),numel(ind),1);
xlrad = bsxfun(@plus,bsxfun(@times,-lmks(3,:),cosine),lmks(1,:));
ylrad = bsxfun(@plus,bsxfun(@times,-lmks(3,:),sine),lmks(2,:));
xorad = bsxfun(@plus,bsxfun(@times,-obs(3,:),cosine),obs(1,:));
yorad = bsxfun(@plus,bsxfun(@times,-obs(3,:),sine),obs(2,:));
xvrad = bsxfun(@plus,bsxfun(@times,sen.rad,cosine),pose(1,:));
yvrad = bsxfun(@plus,bsxfun(@times,sen.rad,sine),pose(2,:));

%--------------------------------------------------------------------------
% Output Results

figure(1); clf;
subplot(7,11,[45:50,67:72]);
plot(xorad,yorad,'Color','k','LineWidth',5); hold on;
plot(xlrad,ylrad,'Color','k','LineWidth',5); 
scatter(goal(1),goal(2),70,'r','filled');
for ind = 1:count-1
    if ishandle(curpos)
        delete(curpos);
    end
    if ishandle(currad)
        delete(currad);
    end
    tri = (rot(:,:,ind)*triangle).*dim + repmat(pose(:,ind),1,4);
    subplot(7,11,[1:6,23:28]); hold on;
    imagesc([0,map.length],[0,map.width],grid(:,:,ind).');
    plot(tri(1,:),tri(2,:),'Color','c','LineWidth',1); hold off;
    axis([0,map.length,0,map.width]); title('Stored Map');
    subplot(7,11,[45:50,67:72]); hold on  
    curpos = plot(tri(1,:),tri(2,:),'Color','c','LineWidth',1);
    currad = plot(xvrad(:,ind),yvrad(:,ind),'Color','b','LineWidth',1);    
    axis([0,map.length,0,map.width]); title('World');
    subplot(7,11,[8:11,30:33]);
    semilogy(pdiag(1:6,1:ind+1)'); title('Certainty of States');
    legend('N','E','D','\theta','\phi','\psi');
    subplot(7,11,[52:55,74:77]);
    semilogy(pdiag(7:12,1:ind+1)');
    title('Certainty of State Derivatives');
    legend('u','v','w','p','q','r'); 
    pause(sen.time);
end
disp(['Simulation Time: ',num2str(time), ' [s]']);

end