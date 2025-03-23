function [map, myline] = helperaddnewscan(map, scan, pose, hits)

%------------------- Line detection (one scan)--------------------------
% this function is way similar to helperinitialization.m
% this process current scan, transforms it to world frame -> detects line
% -> generates dense points 
% then adds those points + features into map
scan_w = helpertransformrobottoworldcoordinate(scan, pose); 
newpts = scan_w(hits>0.3, :);
mclust = helperclusteringfirstlevel(newpts, 0.3);               % 1st-level cluster
sclust = helpercornerdetection(mclust);                         % Corner Detect    
myline = helperpcalinewithdeletingandmerging(sclust);           % PCA Line fitting
tempPts = helperexpandlinepointswithlinefeature(myline.point);    
npts = size(tempPts,1);

k = length(map.keyscans); 
iEnd = map.keyscans(end).iEnd;

if isempty(tempPts)
    map.line(k+1).point = [];
    map.line(k+1).decision = [];
    map.keyscans(k+1).pose = pose;
    map.keyscans(k+1).iBegin = iEnd;
    map.keyscans(k+1).iEnd = iEnd;
    map.keyscans(k+1).loopTried = false;
    map.keyscans(k+1).loopClosed = false;
    return;
end

% points
map.points(iEnd+1:iEnd+npts,1:2) = tempPts;

% keyscans
map.line(k+1).point = myline.point;
map.line(k+1).decision = myline.decision;
map.keyscans(k+1).pose = pose;
map.keyscans(k+1).iBegin = iEnd+1;
map.keyscans(k+1).iEnd = iEnd + npts;
map.keyscans(k+1).loopTried = false;
map.keyscans(k+1).loopClosed = false;

