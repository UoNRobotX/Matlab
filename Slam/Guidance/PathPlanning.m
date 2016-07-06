function [desired] = PathPlanning(goal,obstacles,est,path,width,boat)
%This function is the entire path planning algorithm. It calculates the
%potential path, and outputs the desired new point for the boat to travel

% Parameters

att = path.att;
rep = path.rep;
bar = path.bar;
rad = path.rad;
qstar = path.qstar;
time = path.time;
amax = boat.amax;
vmax = boat.vmax;

% Goal Position and Current Estimated States

goalx = goal(1);
goaly = goal(2);
posx = est(1);
posy = est(2);
velpx = est(7);
velpy = est(8);
thetap = est(6);
omegap = est(12); 

% Attraction quadratic
gradatt = att*[posx-goalx, posy-goaly]';                                              

% Barrier hyperbolic function 
gradbar = bar*[0,(1/((width-posy)^2))-(1/(posy^2))]';

% Repulstion function
gradrep = [0 0]';
if ~isempty(obstacles)
    obs = obstacles(1:2,:);
    dist = obstacles(3,:);
    col = size(obstacles,2);
    dist = dist - repmat(rad,1,col);
    dob = dist(1);
    for count = 2:length(dist)
        if dist(count) < dob
            dob = dist(count);
        end
    end
    if dob <= qstar
        const = (rep/(dob^2))*((1/qstar)-(1/dob));
        for count = 1:col
            if dist(count) <= 0
                gradob = inf;
            else
                gradob = (1/dist(count))*[posx-obs(1,count), posy-obs(2,count)]';
            end
            gradrep = gradrep + const*gradob;
        end
    end
end
    
% Total
gradient = gradatt + gradrep + gradbar;

% Attractive hessian
hessatt = att*eye(2);                                                  
    
% Repulsive hessian
hessrep = zeros(2);
if ~isempty(obstacles)
    obs = obstacles(1:2,:);
    dist = obstacles(3,:);
    col = size(obstacles,2);
    dist = dist - repmat(rad,1,col);
    dob = dist(1);
    for count = 2:length(dist)
        if dist(count) < dob
            dob = dist(count);
        end
    end
    if dob <= qstar
        const = (rep/(dob^2))*((1/qstar)-(1/dob));
        for count = 1:col
            hessdob = zeros(2);
            if dist(count) <= 0
                hessrep = inf*ones(2);
            else
                gain = const/(dist(count)^3);
                hess11 = (obs(2,count)-posy)^2;
                hess12 = (obs(2,count)-posy)*(posx-obs(1,count));
                hess21 = (posy-obs(2,count))*(obs(1,count)-posx);
                hess22 = (obs(1,count)-posx)^2;
                hessdob = gain*[hess11,hess12;hess21,hess22];
            end
            hessrep = hessrep + const*hessdob;
        end  
    end
end

% Barrier hessian
hessbar = [0 0; 0 2*bar*(((width-posy)^-3)+(posy^-3))];

% Total hessian
totalhess = hessatt + hessrep + hessbar;

% Desired Direction 
direct = -totalhess*gradient;

% Line slope and intercept
slope = direct(2)/direct(1);
if abs(slope) ~= Inf
    if isnan(abs(slope))
        slope = 0;
        intercept = posx;
    else
        intercept = posy - (slope*posx);
    end
else
    slope = abs(slope);
    intercept = posx;
end
normdir = [direct(1)/norm(direct(1)) direct(2)/norm(direct(2))]';
normvel = [velpx/norm(velpx) velpy/norm(velpy)]';

for count = 1:2
    if isnan(normdir(count))
        normdir(count) = 0;
    end
    if isnan(normvel(count))
        normvel(count) = 0;
    end
end

% Radii of circles constraining new point and origin of accel circle
radacc = 0.5*amax*(time^2);
radvel = vmax*time;
accx = posx + (velpx*time);
accy = posy + (velpy*time);

% All possible intersections
[intx, inty] = circcirc(posx,posy,radvel,accx,accy,radacc);
[linax, linay] = linecirc(slope,intercept,accx,accy,radacc);
[linvx, linvy] = linecirc(slope,intercept,posx,posy,radvel);

% Determine which intersection is further away from original position
if isfinite(linax)
    lenone = sqrt((linax(1)-posx)^2 + (linay(1)-posy)^2);
    lentwo = sqrt((linax(2)-posx)^2 + (linay(2)-posy)^2);
    if lenone >= lentwo
        maxa = [linax(1) linay(1)];
        mina = [linax(2) linay(2)];
    else
        maxa = [linax(2) linay(2)];
        mina = [linax(1) linay(1)];
    end
end

% Determine which intersection is closer to endpoint of velocity vector
if ~isnan(linvx)
    lenone = sqrt((linvx(1)-accx)^2 + (linvy(1)-accy)^2);
    lentwo = sqrt((linvx(2)-accx)^2 + (linvy(2)-accy)^2);
     if lenone <= lentwo
        maxv = [linvx(1) linvy(1)];
        minv = [linvx(2) linvy(2)];
    else
        maxv = [linvx(2) linvy(2)];
        minv = [linvx(1) linvy(1)];
    end
end


% Determine if direction of newton positive respect to velocity vector
if normdir(1) == normvel(1)
    if normdir(1) == 0
        if normdir(2) == normvel(2)
            oppose = 0;
        else
            oppose = 1;
        end
    else
        oppose = 0;
    end
else
    if (normdir(1) == 0) || (normvel(1) == 0)
        if normdir(2) == normvel(2)
            oppose = 0;
        else
            oppose = 1;
        end
    else
        oppose = 1;
    end
end

% Determine new position in space
if ~isnan(intx)
    % Intersection angles from origin
    intangleone = atan2((inty(1)-posy),(intx(1)-posx));
    intangletwo = atan2((inty(2)-posy),(intx(2)-posx));
    if intangleone < 0
        intangleone = intangleone + (2*pi);
    end
    if intangletwo < 0
        intangletwo = intangletwo + (2*pi);
    end
    % Determine maximum and minimum angle
    if intangleone >= intangletwo
        max = intangleone;
        min = intangletwo;
    else
        max = intangletwo;
        min = intangleone;
    end
    % Determine if velocity vector within angles
    velangle = atan2(velpy,velpx);
    if velangle < 0
        velangle = velangle + 2*pi;
    end
    if (velangle > max) || (velangle < min)
        flag = 1;
    else
        flag = 0;
    end
    % Determine if newton line within contraints
    directangle = atan2(direct(2),direct(1));
    if directangle < 0
        directangle = directangle + 2*pi;
    end
    constrained = 1;
    if flag
        if (directangle < max) && (directangle > min)
            constrained = 0;
        end
    else
        if (directangle > max) || (directangle < min)
            constrained = 0;
        end
    end
    if constrained
        if ~oppose
            xnew = maxv(1);
            ynew = maxv(2);
        else
            xnew = mina(1);
            ynew = mina(2);
        end
    else
        if ~isnan(linax)
            if ~oppose
                xnew = maxa(1);
                ynew = maxa(2);
            else
                xnew = mina(1);
                ynew = mina(2);
            end
        else
            % Find direction of perpendicular line
            slope = direct(2)/direct(1);
            if abs(slope) ~= Inf
                if isnan(abs(slope))
                    dirx = sign(velpx);
                    diry = 0;
                    perpdir = [diry -dirx]';
                else
                    direction = (direct(2)*velpy)-(direct(1)*velpx);
                    if direction < 0
                        perpdir = [-direct(2) direct(1)]';
                    else
                        perpdir = [direct(2) -direct(1)]';
                    end
                end
            else
                dirx = 0;
                diry = sing(velpy);
                perpdir = [diry -dirx]';
            end
            % Calculate new position for boat
            perpdist = (perpdir/norm(perpdir))*radacc;
            xnew = accx + perpdist(1);
            ynew = accy + perpdist(2);
        end
    end
else
    if radacc < radvel
        if ~isnan(linax)
            if ~oppose
                xnew = maxa(1);
                ynew = maxa(2);
            else
                xnew = mina(1);
                ynew = mina(2);
            end
        else
            % Find direction of perpendicular line
            slope = direct(2)/direct(1);
            if abs(slope) ~= Inf
                if isnan(abs(slope))
                    dirx = sign(velpx);
                    diry = 0;
                    perpdir = [diry -dirx]';
                else
                    direction = (direct(2)*velpy)-(direct(1)*velpx);
                    if direction < 0
                        perpdir = [-direct(2) direct(1)]';
                    else
                        perpdir = [direct(2) -direct(1)]';
                    end
                end
            else
                dirx = 0;
                diry = sing(velpy);
                perpdir = [diry -dirx]';
            end
            % Calculate new position for boat
            perpdist = (perpdir/norm(perpdir))*radacc;
            xnew = accx + perpdist(1);
            ynew = accy + perpdist(2);
        end
    else
        if ~oppose
            xnew = maxv(1);
            ynew = maxv(2);
        else
            xnew = minv(1);
            ynew = minv(2);
        end
    end
end

point = [xnew ynew]';

% Calculate desired values
xnext = point(1);
ynext = point(2);
velx = (xnext-posx)/time;
vely = (ynext-posy)/time;
accx = (velx-velpx)/time;
accy = (vely-velpy)/time;
theta = atan2((ynext-posy),(xnext-posx));
omega = (theta-thetap)/time;
zeta = (omega-omegap)/time;

desired = [xnext ynext theta velx vely omega accx accy zeta]';

end