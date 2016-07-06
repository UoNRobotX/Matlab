function [y,yhat] = ObMeasurements(MapY,GridMap,map,rnBN,Limit)
%This function returns vectors of distances between the boat and the
%seen obstalces (y), and the distance between the boat and the stored 
%obstalces (yhat)

% Parameters
res = map.res;
measure = zeros(360,2);
for count = 1:2
    if count == 1
        MapE = MapY;
        thres = map.inc;
    else
        MapE = GridMap;
        thres = map.thres;
    end
    for Y = Limit(3):Limit(4)
        for X = Limit(1):Limit(2)
            if MapE(X,Y) >= thres
                Xdis = res*(X-1/2)-rnBN(1);
                Ydis = res*(Y-1/2)-rnBN(2);
                Ang = round(res*sqrt(2)/sqrt(Xdis^2+Ydis^2)*180/pi);
                AngStart = round((atan2(Ydis,Xdis)*180/pi) - Ang/2);
                % Limit Angle within [0,2Pi]
                if AngStart <= 0
                    AngStart = 360 + AngStart;
                elseif AngStart > 360
                    AngStart = AngStart - 360;
                end
                % Work Through the Arc
                for loop = AngStart:(AngStart+Ang)
                    index = loop;
                    if index > 360
                        index = loop - 360;
                    end
                    if (measure(index,count) > sqrt(Xdis^2+Ydis^2))||(measure(index,count) == 0)
                        measure(index,count) = round(sqrt(Xdis^2+Ydis^2));
                    end                
                end       
            end
        end
    end
end

y = measure(:,1);
yhat = measure(:,2);

end