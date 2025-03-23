% goal: 1) checking if current pose is near a previously visited location.
% 2) running brute-force scan matching
% if match is found, correct current pose, b) replace current scan's map
% data (points + lines ) c) set loop closure flag 
% improves map accuracy and reduces drift over time. 

function [map, matchScan] = helperloopclosuredetect(map, scan, hits, Tmax, Rmax, pixelSize)
% Input:
%   map:  Current SLAM map (map.points, map.keyscans, etc)
%   scan: current scan in lidar frame
%   hits: result of previous scan matching (used as reference)
%   minOverlap: minimum number of overlapping points required to consider a
%   match 
%   maxAngleDiff: maximum angle diff allowed for a match 
%   pixelSize: resolution for building occupancy grids. 

% searches old keyframes for a place that robot might have visited
% tries scan matching against that area
% if matched, update's a)robot pose b) replace current keyframe data c)
% marks loopclosed = true


k = length(map.keyscans);  % current total number of key scan
pose = map.keyscans(end).pose; % current pose (need modification)

%---------- Condition 1: Distance > 50 ----------
% find a past key frame with 20 m distance travelled
% this avoids matching against recent nearby keyframes - loop closure only
% makes sense after long travel.

S = 0;
% s = distance. stop this loop when distance is 20 m
while k > 1 && S < 20
    k = k-1;
    % subtracting distances between poses.

    dT = helperposedifference(map.keyscans(k).pose, map.keyscans(k+1).pose);
    S  = S + norm(dT(1:2)); 
end

%---------- Condition 2: Displacement < Tmax ----------
% after finding far enough (20 M) key frame K we check is any of them close to
% current pose ? 
% if yes loop detected 
L = 0;
for i = k : -1 : 1
    dp = helperposedifference(map.keyscans(i).pose, pose);
    if norm(dp(1:2)) < Tmax + 1 %Tmax = 4. 
        L = i;
        matchScan = L;
        break;
    end
end
if L < 1 
    % exit early no loop closure. 
    matchScan = 0;
    return;
end

disp('Loop detected');

% =============== Loop Closure Detection ===============
% Extract a refrence map around current scan

% get points upto matched key frame

% matchScan index of old key frame matched in loop closure
% iEnd -> ending index of points in map.points that belong to that key
% frame
% ndear = stores all map points from beginging up to past keyframe
% keyframe : it stores metadata about one important LiDAR scan used to
% build map. keyscan includes
% pose: robot's estimated pose [x, y, theta] when scan was taken. 
% iBegin, iEnd : start and end indices in map.points corresponding to this
% scan. 
% loopClosed : whether loop closure has been applied to this scan
% loopTried: whether loop closure was tried on this scan?

% map.points actual points used to build global 2D map
% these are dense points interpolated from fitted lines -- not from raw
% LiDAR ranges !
% every time you add a key frame ( helperaddnewscan or
% helperinitialization) it generates interpolated map points from detected
% lines

% keyscans are scalar
ndear = map.keyscans(matchScan).iEnd;

% extract a local map around current pose, subset of full map. includes all
% points in map that were created upto and including matchscan
% used as reference map for loop closure.

dearPoints = map.points(1:ndear, :);

% extracts a local submap around current pose Tmax+ 4 is bounding box size
% used to build 
refMap  = helpermapextractlocalmap(dearPoints, scan, pose, Tmax+4);
% refMap  = helpermapextractlocalmap(map.points, scan, pose, Tmax+4);

% searches old keyframes that we
% Hits : how well current scan matched last submap 
% this thresholds are used to decide whether new match is better
% scoreThresh: total score must be less than this.
% countThresh: number of points that missed matching must be less than
% this. 
scoreThresh = sum(hits) * 0.8;
countThresh = sum(hits==0) * 0.6;       

% converts refMap into an occupancy grid (regGrid.occGrid) and distance
% map(refGrid.metricMap)

refGrid = helperoccugridcreate(refMap, pixelSize);

% resolution for grid search +- 0.05 in x, +- 0.05m in y, +- 0.5 degree in
% theta
resol = [0.05, 0.05, deg2rad(0.5)];

% inside brute force scan matching 
% tries every (x, y, theta) withing small bounds
% transforms scan, drops it onto grid.
% uses metricmap (distance transform) to get score. 
% best pose one with lowest score 
% used instead of hill climbing because its more robust in loop closure(
% global search ) 
[bestPose, bestHits] = helpermatchingbrute(refGrid, scan, pose, resol, Tmax/Tmax, Rmax);

% bestPose : pose where scan best aligns with refMap
% bestHits : score values at that pose. 

% check two conditions 
% sum(bestHits < 4) count of scan points matched closely less than 4 meters

if sum(bestHits<4) > countThresh && sum(bestHits) < scoreThresh
    disp('Loop closure');
    pose = bestPose;
    hits = bestHits;
    pause(1);
end

% reprocess current scan with updated pose. 
% re run entire mapping pipeline. 
% rebuild keyframe using corrected pose or original pose

% transform scan to world frame
scan_w = helpertransformrobottoworldcoordinate(scan,pose);

% reprocess scan (SLAM FRONTEND)
% runs standard feature pipeline. cluster scan points
mclust = helperclusteringfirstlevel(scan_w,0.3);             % 1st-level cluster    
% detect corners -> refine clusters
sclust = helpercornerdetection(mclust);                      % Corner Detect    

% fit line segments to cluster
myline = helperpcalinewithdeletingandmerging(sclust);  % PCA Line fitting

% densify line features into map points.
tempPts = helperexpandlinepointswithlinefeature(myline.point); npts = size(tempPts,1);

%-------------------- Update Map --------------------
% delete old keyframe data. remove points and line segments that were added
% before loop closure correction. keep the map clean.
a = map.keyscans(end).iBegin;
b = map.keyscans(end).iEnd;
front = map.keyscans(end-1).iEnd;
map.points(a:b,:) = []; 
map.line(end) = [];

% Update
% replace map with updated data from corrected pose. 
% mark loopclosed = true so it isn't checked again. 
% re-add corrected, map points -> line geometry -> metadata (centroid,
% orientation)
k = length(map.keyscans);
map.points(front+1:front+npts,1:2) = tempPts;
map.line(k).point = myline.point;
map.line(k).decision = myline.decision;

% updates pose, point indices, marks this key scan s "loop-closed" so it
% not revisited again.

map.keyscans(end).pose = pose;
map.keyscans(end).iBegin = front + 1;
map.keyscans(end).iEnd = front + npts;
map.keyscans(end).loopClosed = true;


% summary 
% travel check : go back until robot has moved enough
% find nearby past pose : check if robot has returned to a place. 
% extract local map : around matched past keyframe
% brute-force match : try matching current scan to past location
% verify match : if overlap is strong; its a loop closure 
% re-process L run full line detection pipeline with cirrected pose
% update map : replace old data mark loop closure. 


% suppose you are at 100th frame
% then you walk back until you find that from k = 100 to k = 75 the
% cumulative traveled distance is 20 meters

% now you are checking whether any earlier pose ( frames between 1 to 75)
% is spatially near current pose (100) 

% dp = pose pose_100 - pose_I, this is Euclidean distance calculation. 
% gives output for how far is robot pose from pose at keyframe i
% and compare it Tmax + 1 = 5 meters 
% if any pose from frames 1 to 75 is within 5 meters of current pose ->
% that's a trigger for loop closure. 3.5 < 5.

% 
    
    
    
    
    
    
    
    
    