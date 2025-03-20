clear; close all; clc;
%--------------------------- Setting variable --------------------------
lidar = helperinitializelidarparameters(1);
cfig = figure(1);
lidar_data = load('dataset.mat');
N = size(lidar_data.times, 1);

% Map parameters 
borderSize    = 1;            
pixelSize     = 0.2;          
miniUpdated   = false;         
miniUpdateDT  = 0.1;         
miniUpdateDR  = deg2rad(5);   

% Scan matching parameters
fastResolution  = [0.05; 0.05; deg2rad(0.5)]; 
bruteResolution = [0.01; 0.01; deg2rad(0.1)];

% Create an empty map
map.points = [];
map.keyscans = [];    % keyframe
pose = [0; 0; 0];
path = pose;

% PoseGraph
poseGraph = helperclassposegraph();
infm1 = [20 0 0;0 20 0;0 0 100000];
infm2 = [100 0 0;0 100 0;0 0 1000000];

%============================== SLAM Part ==============================
tic;
for scanIdx = 1 : 1 : N
    
    disp(['scan ', num2str(scanIdx)]);
    scan = helperreadframe(lidar_data, scanIdx, lidar, 24);
 %------------------------ Initialize ------------------------   
    if scanIdx == 1
        [~, map] = helperinitialization(scan,pose,map);
        poseGraph.insertNode(1,pose);
        miniUpdated = true;  continue;
    end
        
 %------------------------ Pose initial guess ------------------------   
    if scanIdx > 2
        pose_guess = pose + helperposedifference(path(:,end-1), pose);
    else
        pose_guess = pose;
    end
    
%------------------------ Matching scan2submap ------------------------
    if miniUpdated
        localMap = helpermapextractlocalmap(map.points,scan,pose,borderSize); 
        if isempty(localMap)
            error("Too much error !");
        end
        gridMap1 = helperoccugridcreate(localMap, pixelSize);
        gridMap2 = helperoccugridcreate(localMap, pixelSize/2);
        % match current pose with the last keyscan's submap
        [pose, ~] = helpermatchingfast(gridMap1, scan, pose_guess, fastResolution);
    else
        [pose, ~] = helpermatchingfast(gridMap2, scan, pose_guess, fastResolution);
    end
    % Refine the pose using smaller pixels
    [pose, hits] = helpermatchingfast(gridMap2, scan, pose, fastResolution/2);
%     [pose, hits] = helpermatchingbrute(gridMap2, scan, pose, bruteResolution,0.1,deg2rad(0.5));
   
%------------------------ Update Map & Pose------------------------
    dp = abs(helperposedifference(map.keyscans(end).pose, pose));
    
    if dp(1)>miniUpdateDT || dp(2)>miniUpdateDT || dp(3)>miniUpdateDR
        miniUpdated = true;        
        [map,~] = helperaddnewscan(map, scan, pose, hits);
        % update pose-graph
        nkey = length(map.keyscans); % number of keyscan
        poseGraph.insertNode(nkey,pose);        
        
        if helperloopclosure(map)
            map.keyscans(end).loopTried = true;
            [map, matchScan] = helperloopclosuredetect(map, scan, hits, 4, pi/6, pixelSize);
            pose = map.keyscans(end).pose;
            % update pose-graph
            poseGraph.insertNode(nkey, pose);              
        end
                
    else
        miniUpdated = false;
    end
   
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