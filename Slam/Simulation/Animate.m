function [time] = Animate(fig,landmarks,obstot,gridmap,boat,map,dis,ang,goal,loop,pdiag,path)%,points)
%This function does an animation of the boat through the map

% Parameters
res = map.res;
length = map.length;
width = map.width;      
linwid = fig.linwid; 
offset = fig.offset;  
chars = fig.chars;
start = fig.start;
ends = fig.finish;
lengboat = boat.length/2;
widthboat = boat.width/2;
view = path.certain;
colour = fig.boatcolour;
radcol = fig.radcolour;
starttime = toc;
init = dis(:,1);

% Animation
figure(1)
clf
subplot(4,11,[1:6,12:17])
imagesc(gridmap.');
hold on
plot((dis(1,loop)/res)+1,(dis(2,loop)/res)+1,'or')
[quivx,quivy] = pol2cart(ang(1,loop),1);
hold on
quiver((dis(1,loop)/res)+1,(dis(2,loop)/res)+1,quivx,quivy,'-r')
hold on
obsx = obstot(1,:);
obsy = obstot(2,:);
radii = -obstot(3,:);
for count = 1:size(obstot,2)
    xrad = radii(count)*cos(0:0.1:2*pi) + obsx(count);
    yrad = radii(count)*sin(0:0.1:2*pi) + obsy(count);
    plot(xrad/res,yrad/res,'LineWidth',fig.linwid,'Color',[1 1 1]);
    hold on    
end
title('Map: Mirrored')
subplot(4,11,[23:28,34:39])
% Background map
plot(init(1),init(2),'o');
hold on
plot(goal(1),goal(2),'o');
hold on
for count = 1:size(chars,1)
    text(-offset*length,chars(count),num2str(start(count)));
    text((1+offset)*length,chars(count),ends(count));
end
set(gca,'YTick',[0 width])
axis([0 length 0 width]);
xlabel('Real World');
hold on
% Plot Landmarks
obsx = landmarks(1,:);
obsy = landmarks(2,:);
radii = -landmarks(3,:);
for count = 1:size(landmarks,2)
    disvect = abs(landmarks(1:2,count) - dis(:,loop));
    dist = sqrt(sum(disvect.^2));
    if dist <= path.certain
        xrad = radii(count)*cos(0:0.1:2*pi) + obsx(count);
        yrad = radii(count)*sin(0:0.1:2*pi) + obsy(count);
        plot(xrad,yrad,'LineWidth',fig.linwid,'Color',fig.obscolour);
        hold on  
    end
end
% Plot Obstacles
obsx = obstot(1,:);
obsy = obstot(2,:);
radii = -obstot(3,:);
for count = 1:size(obstot,2)
    disvect = abs(obstot(1:2,count) - dis(:,loop));
    dist = sqrt(sum(disvect.^2));
    if dist <= path.certain
        xrad = radii(count)*cos(0:0.1:2*pi) + obsx(count);
        yrad = radii(count)*sin(0:0.1:2*pi) + obsy(count);
        plot(xrad,yrad,'LineWidth',fig.linwid,'Color',fig.obscolour);
        hold on  
    end
end
% % Plot Lumpong Lines
% if ~isempty(points)
%     pointx = points(1,:);
%     pointy = points(2,:);
%     for count = 1:size(points,2)
%         disvect = abs(points(1:2,count) - dis(:,loop));
%         dist = sqrt(sum(disvect.^2));
%         if dist <= path.certain
%             plot(pointx(count),pointy(count),'LineWidth',fig.linwid,'Color',fig.lumpcolour);
%             hold on  
%         end
%     end
% end
% Plot Boat
triangle = [-1 1 0 -1; -1 -1 1 -1];
angle = ang(loop) - pi/2;
pos = dis(:,loop);
tri = [cos(angle) -sin(angle); sin(angle) cos(angle)]*triangle;
tri(1,:) = tri(1,:)*lengboat + pos(1);
tri(2,:) = tri(2,:)*widthboat + pos(2);
plot(tri(1,:),tri(2,:),'Color',colour,'LineWidth',linwid);
hold on;
xrad = view*cos(0:0.1:2*pi) + dis(1,loop);
yrad = view*sin(0:0.1:2*pi) + dis(2,loop);
plot(xrad,yrad,'LineWidth',linwid,'Color',radcol);
hold on;
% Plot Confidences
subplot(4,11,[8:11,19:22])
semilogy(pdiag(1:6,:).')
title('Certainty of States')
legend('X','Y','Z','\theta','\phi','\psi');
subplot(4,11,[30:33,41:44])
semilogy(pdiag(7:12,:).')
xlabel('Certainty of State Derivatives')
legend('V_{x}','V_{y}','V_{z}','V_{\theta}','V_{\phi}','V_{\psi}');
fintime = toc;
time = fintime-starttime;
end