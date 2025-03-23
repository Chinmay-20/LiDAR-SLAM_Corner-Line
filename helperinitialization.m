function [line, map] = helperinitialization(scan, pose, map)

    % line detection
    scan_w = helpertransformrobottoworldcoordinate(scan,pose);
    mclust = helperclusteringfirstlevel(scan_w,0.3);            % 1st-level cluster
    sclust = helpercornerdetection(mclust);                  % Corner Detect
    line = helperpcalinewithdeletingandmerging(sclust); % PCA Line fitting
    tempPts = helperexpandlinepointswithlinefeature(line.point);
    
    % tempPts is Nx2 matrix containing dense interpolated (x,y) points
    % along detected lines 
    % update map        
    map.points(:,1:2) = tempPts; % stores map's point cloud for visualization, stores dense 2D points in map
    
    % determines index of new key frame. for first frame it will be 1
    k = length(map.keyscans)+1;
    
    % stores line raw line segment detected from PCA
    % store start and end points of each line segment
    map.line(k).point = line.point;
    % slope m, intercept n, centroid -- used for merging and visualization
    map.line(k).decision = line.decision;

    % stores robot's current pose 
    map.keyscans(k).pose = pose;

    % this indexes (iBegin & iEnd) where this scan's points live inside
    % map.points
    % later if you want to extract points from this keyframe
    % map.points(iBegin:iEnd, :)
    map.keyscans(k).iBegin = 1;
    map.keyscans(k).iEnd = size(tempPts,1);
    
    % since this is first frame it considered "closed" ( no loop possible)
    map.keyscans(k).loopClosed = true;
    % no attempt to close loops has been made yet
    map.keyscans(k).loopTried = false;

% full pipeline (first frame only)
% 1. read: Lidar data. one scan 1079 values
% 2. preprocess: remove bad points (too close, too far). convert from polar
% to cartesian coordinates -> gives points in robot frame. 
% 3. transform to world coordinates: use robot pose (x,y, theta) to convert
% scan points from robot frame -> world frame. now all points are aligned
% in gloabl map
% 4. cluserting (1st level): group nearby points using distance threshold(
% 0.3 m). filters our tiny / noisy cluster
% 5. corner detection (2nd level): further split each cluster at corner
% points. Uses PCA and point-to-line distance to find break points
% 6. Line Fitting (PCA) : fit a line to each sub cluster using PCA. remove
% poor fis (low eigen value ratio). merges lines with similar orientation
% and close centroids)
% 7. generate dense map points: for each line interpolate 100+ evenly
% spaced points between endpoints. these are stored in map.points for
% visualization or occupancy mapping. 
% 8. update map structure: store line.point and line.decision, pose,
% keyframe info (start / end indices in point list). loop closure flags. 
% read data -> preprocess ( remove bad points. polar -> cartesian coordinates), -> if it is first frame -> transform robot coordinate in pov of world coordinate frame -> apply clustering to point clouds -> detect corners and lines in clusters -> merge adjacent lines across all cluster which have similar centroids & orientation ->  then for each line generate 100+ points to plot in map 
