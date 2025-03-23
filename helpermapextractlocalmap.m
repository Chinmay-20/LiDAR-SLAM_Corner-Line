% Extract a local map according to current scan position so that scan
% matching happens only in local area. 

% comes in this function only if map is updated 03/21/25 
% goal of this function is to extract local map based on latest
% correct pose and not based on pose guess.

% function is called at frame N to extract a local submap from full map
% created up to frame N - 1.
% this way we use only small area of gloabl map near the current pose for
% scan matching  -- keeping things fast and focused. 

function localMap = helpermapextractlocalmap(point,scan,pose,borderSize)
% Input
%   point: previously stored map points (map.points)
%   scan : current scan in robot frame
%   pose : robot's current estimated pose [x, y, theta]
%   borderSize :how much area around scan to include in local map.
% Output
%   localMap:local map around current pose

% what is local map ? 
% assume point as huge point cloud 
% we don't want to do scan matching against all of it - that's slow
% instead we take current scan
% estimate where it lands in world frame 
% cut out a box around that spot. 
% this box is local map

% transform current scan in robot to world coordinate 
scan_w = helpertransformrobottoworldcoordinate(scan,pose);
% scan_w real-world position of current scan points. 

% Set top-left & bottom-right corner
% calculates a bounding box around scan ( margin = bordersize)

% scan_w(:, 1) -> all X coordinates of current scan points
% scan_w(:, 2) all Y coordinates
% finds smallest and largest X and Y values and then expand those by
% bordersize
% box centered around where the scan hits.
% why are we calling min and max after subtracting border size on all x and
% y values stored in map.points ?
% Ans: so we are subtratcing bordersize from current scan x and y
% coordinates. then taking min and max of shifted points. 
% scan_w(:, 1) = [ 4, 5, 6]
% scan_w(:, 2) = [10, 11, 12]
% bordersize = 1
% minX = min ([4-1, 5-1, 6-1]) = 3
% maxX = max([4+1, 5+1, 6+1]) = 7
% minY = min([10-1, 11-1, 12-1]) = 9
% maxY = max([10+1, 11+1, 12+1]) = 13
% so box belongs to X = [ 3, 7], y = [9, 13]
minX = min(scan_w(:,1) - borderSize); % left
minY = min(scan_w(:,2) - borderSize); % bottom
maxX = max(scan_w(:,1) + borderSize); % right
maxY = max(scan_w(:,2) + borderSize); % top

% selects existing map points that fall in this bounding box.
% logical vector (true/false for each point)

isAround = point(:,1) > minX...
         & point(:,1) < maxX...
         & point(:,2) > minY...
         & point(:,2) < maxY;

% keeps only those map points that lie within bounding box.
localMap = point(isAround, :);

% this keeps ONLY points from map.points that fall inside bounding box
% giving you local map for scan matching 

% returns a local subset of map this helps limit search space and improves
% scan matching speed.


% so for first frame map created, from second frame first pose_guess is calculated, for second frame map as map build in
% previous frame, local map is created, from that local map occupancy grids
% are calculated, from ocupancy grids correct pose are calculated
% pose difference between accurate pose and last pose stored in map is
% calculated if any difference in x, y or translation is greater than
% threshold map is updated. 
% from now on whenever 