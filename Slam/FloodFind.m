function [obsmap,cells] = FloodFind(obsmap,obscount,gridmap,thres,count,loop,cells,res)
%This function returns the a new map outlining which cells correspind to
%which obstalces via numbers, as well as returning the centre points of
%these cells

for xindex = -1:1
    for yindex = -1:1
        xcell = count + xindex;
        ycell = loop + yindex;
        if (xcell >= 1)&&(xcell <= size(gridmap,1))&&(ycell >= 1)&&(ycell <= size(gridmap,2))&&(gridmap(xcell,ycell) >= thres)
            X = res*(xcell-1/2);
            Y = res*(ycell-1/2);
            flag = 0;
            for key = 1:size(cells,2)
                if (X == cells(1,key))&&(Y == cells(2,key))
                    flag = 1;
                end
            end
            if ~flag
                obsmap(xcell,ycell) = obscount;
                cells = cat(2,cells,[X;Y;obscount]);
                [obsmap,cells] = FloodFind(obsmap,obscount,gridmap,thres,xcell,ycell,cells,res);
            end
        end
    end
end
end