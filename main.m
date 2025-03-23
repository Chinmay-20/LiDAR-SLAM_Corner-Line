clear; close all; clc;
%--------------------------- Setting variable --------------------------
lidar = helperinitializelidarparameters(1);
cfig = figure(1);
lidar_data = load('dataset.mat');
N = size(lidar_data.times, 1);

% Map parameters 
borderSize    = 1;          % size of boarder around local map to extract           
pixelSize     = 0.2;        % resolution of occupancy grid
miniUpdated   = false;      % flag to track if map was updated this frame
miniUpdateDT  = 0.1;        % threshold for translation update to add key frame
miniUpdateDR  = deg2rad(5); % threshold for rotation update

% Scan matching parameters
fastResolution  = [0.05; 0.05; deg2rad(0.5)];  % step sizes (x,y,theta for fast scan matching [3x1 vector]
bruteResolution = [0.01; 0.01; deg2rad(0.1)];   % step sizes for brute force scan matching. [ 3x1 vector]

% Create an empty map
% map is struct
map.points = [];        % struct SLAM map with fields .points, .leyscans, .line
map.keyscans = [];      % keyframe
pose = [0; 0; 0];       % current robot pose [x, y, theta]
path = pose;            % history of all poses (used for plotting & estimating)

% PoseGraph
poseGraph = helperclassposegraph();         % stores pose grpah for optimization (loop closure)

% not used yet
infm1 = [20 0 0;0 20 0;0 0 100000];
infm2 = [100 0 0;0 100 0;0 0 1000000];

%============================== SLAM Part ==============================
tic;
for scanIdx = 1 : 1 : N
    
    % disp(['scan ', num2str(scanIdx)]);
    scan = helperreadframe(lidar_data, scanIdx, lidar, 24);
 %------------------------ Initialize ------------------------   
    if scanIdx == 1
        [~, map] = helperinitialization(scan,pose,map);
        % updated map returned replaces input
        % map contains first scan processed points, detected line segments,
        % keyscan metadata (pose, start/end indices, flags)
        % start with empty map and then initialize it wik meaningful data
        
        % creates a node in pose graph
        % id = 1
        % pose = [0, 0, 0]
        poseGraph.insertNode(1,pose);
        
        miniUpdated = true;  % map was updates in this frame. because we added first frame to map.
        continue;
        % pose is [0; 0; 0;] (origin) 
        % no pose guessing
        % no scan matching
        % map is initialized with first scan
        % pose graph gets its first node then continue. 
    end
        
 %------------------------ Pose initial guess ------------------------   
    if scanIdx > 2
        % from third scan and beyond
        % 1. path(:, end-1) -> robot pose from previous keyframe
        % 2. pose -> current pose (after scan matching from last frame
        % 3. estimated_motion : helperposedifference(path(:,end-1), pose) estimates how much
        % robot moved between previous and current pose. 
        % 4. pose_guess = pose + (estimated motion) : this projects robot's
        % movment forward - a rough prediction of where it will be in next
        % scan. 
        % disp(['scan ', num2str(scanIdx)]);

        % from scan 3 onwards compute motion between last two poses
        % predict next pose: pose_guess = pose + diff.
        % this is used as starting point for scan matching. 
        pose_guess = pose + helperposedifference(path(:,end-1), pose);
    else 
        % if scanIdx == 2 at this point there is only one pose in path -> no motion estimate yet.
        % so we just use current pose. 
        % pose_guess = pose (which is still origin depending on previous
        % loop)
        pose_guess = pose;
        % disp("in else");
    end
    % after pose_guess is estimated , refines pose by aligning current scan
    % with a local map. 
    % aligning is done by helperMatchingfast()
    % pose is updates and stored in path.
%------------------------ Matching scan2submap ------------------------
    if miniUpdated
        % extracted local subset of map to limit search space and increase
        % scan matching speed.
        localMap = helpermapextractlocalmap(map.points,scan,pose,borderSize); 
        
        if isempty(localMap)
            % if scan is too far off from existing map, matching might fail
            % this check prevents that.
            error("Too much error !");
        end

        % create two occupancy grids
        % normal resolution
        gridMap1 = helperoccugridcreate(localMap, pixelSize);
        % higher resolution
        gridMap2 = helperoccugridcreate(localMap, pixelSize/2);
        
        % match current pose with the last keyscan's submap
        % uses a hill-limbing algorithm
        [pose, ~] = helpermatchingfast(gridMap1, scan, pose_guess, fastResolution);
    else
        [pose, ~] = helpermatchingfast(gridMap2, scan, pose_guess, fastResolution);
    end
    % Refine the pose using smaller pixels (finer pixel size), smaller
    % resolution step, search space is centered around updated pose. 
    [pose, hits] = helpermatchingfast(gridMap2, scan, pose, fastResolution/2);

    % result : a more precise pose aligned with local map.
    % pose : more accurate position of robot
    % hits : how well scan matched. 
    % [pose, hits] = helpermatchingbrute(gridMap2, scan, pose, bruteResolution,0.1,deg2rad(0.5));
   
%------------------------ Update Map & Pose------------------------
    % calculate pose difference between current pose and latest keyscan's
    % pose
    dp = abs(helperposedifference(map.keyscans(end).pose, pose));
       
    % check for enough movement, if x or y change more than 0.5m or theta
    % changed more than 0.2 radian, then add new key frame to map
    % if difference is greater than threshold, you add new frame to map
    % now loop closure depends, but when new frame is added , in next frame
    % local map is updated. 
    if dp(1)>miniUpdateDT || dp(2)>miniUpdateDT || dp(3)>miniUpdateDR
        miniUpdated = true;        

        % update map
        [map,~] = helperaddnewscan(map, scan, pose, hits);
        % update pose-graph

        % add new pose to graph used later for optimization.
        nkey = length(map.keyscans); % number of keyscan
        poseGraph.insertNode(nkey,pose);        
        
        % check if loop closure is needed 2 conditions at least 10 key
        % frames , check using variable loopTried 
        % and at least 5 m distance is traveled since last loop closure known by using flag loopClosed(true indicated loop was closed),
        % calculated using posedifference and traversing map from backwards
        % frames 
        if helperloopclosurecheck(map)
            % attempt loop closure. 
            
            map.keyscans(end).loopTried = true; % marks that we tried loop closure, for next 10 frames no loop closure. 
            [map, matchScan] = helperloopclosuredetect(map, scan, hits, 4, pi/6, pixelSize);
            pose = map.keyscans(end).pose;
            % update pose-graph
            poseGraph.insertNode(nkey, pose);              
        end
                
    else
        miniUpdated = false;
    end
    
    % every frame's pose is appended to path for visualization and
    % trajectory tracking
    path = [path  pose];
%----------------------------- Plot -----------------------------
    if mod(scanIdx, 30) == 0        
         helperplottingmap(cfig, map, path, scan, scanIdx);
    end
end

mytime = toc;
disp(['Total time: ',num2str(toc)]);
 
figure(2)
plot(map.points(:,1),map.points(:,2),'k.','markersize',0.2);

% save('point_map.mat','map');