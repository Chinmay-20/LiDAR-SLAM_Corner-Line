% Fast scan matching, note this may get stuck in local minimas

% this is scan-to-map matching and uses pose guess as center of search 
function [pose, bestHits] = helpermatchingfast(gridmap, scan, pose, searchResolution)
% FastMatch:According to the grid map of the current pose, the predicted
%   next pose is optimized to maximize the coincidence between the grid map
%   of the next pose and the grid map of the current pose
% Input
%   scan:Robort Coordinate
%   pose:initial guess
% Output
%   pose:Optimized pose 
%   bestHits:The distance matrix of the current pose
%       grid map corresponding to the best coincidence degree

% uses a hill climbing algorithm 
% try 3 orientations theta +- r
% for each try 3x position and 3y positions

%---------- Grid map information ----------
metricMap = gridmap.metricMap;      % BW map
ipixel = 1 / gridmap.pixelSize;     % number of grids corresponding to 1m
minX   = gridmap.topLeftCorner(1);  % gridmap.topLeftCorner = minXY
minY   = gridmap.topLeftCorner(2);
nRows  = size(metricMap, 1);
nCols  = size(metricMap, 2);

%---------- Set and Initialize ----------
maxIter = 50;                   % maximum iteration
maxDepth = 3;                   
t = searchResolution(1);
r = searchResolution(3);
iter = 0;
depth = 0;
% convert the actual scanned data to grid coordinates in grid map
pixelScan = scan * ipixel;
bestPose  = pose;
bestScore = Inf;

%-------------------- Hill-climbing Method --------------------
while iter < maxIter
    noChange = true;    
    % Rotation
    for theta = pose(3) + [-r, 0, r]    % [theta-r theta theta+r]        
        ct = cos(theta);
        st = sin(theta);
        S  = pixelScan * [ct, st; -st, ct];        
        % Translation
        for tx = pose(1) + [-t, 0, t]
            Sx = round(S(:,1)+(tx-minX)*ipixel) + 1;
            for ty = pose(2) + [-t, 0, t]
                Sy = round(S(:,2)+(ty-minY)*ipixel) + 1;
                
                %----- metric socre -----
                
                isIn = Sx>1 & Sy>1 & Sx<nCols & Sy<nRows;
                ix = Sx(isIn);
                iy = Sy(isIn);                   
                idx = iy + (ix-1)*nRows;    
                
                hits = metricMap(idx);      
                score = sum(hits);
                
                
                %----- update -----
                if score < bestScore
                    noChange  = false;
                    bestPose  = [tx; ty; theta];
                    bestScore = score;
                    bestHits  = hits;
                end                
            end
        end
    end
    
    % No better match was found, increase resolution
    if noChange
        r = r / 2;
        t = t / 2;
        depth = depth + 1;
        if depth > maxDepth
            break;
        end
    end
    
    pose = bestPose;        % pose optimizing
    iter = iter + 1;
    
end
    
        
                
                
                
            
        
        