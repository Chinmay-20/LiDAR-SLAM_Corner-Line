% Robot coordinate converse to world coordinate
function tscan = helpertransformrobottoworldcoordinate(scan, pose)
% Input
%   scan:current in Robot coordinate
%   pose:current pose
% Output
%   tscan:transformed scan

tx = pose(1);
ty = pose(2);
theta = pose(3);

ct = cos(theta);
st = sin(theta);
R  = [ct, -st; st, ct]; 

% transformed scan
tscan = scan * (R'); 
tscan(:,1) = tscan(:,1) + tx;
tscan(:,2) = tscan(:,2) + ty;

% if sensor is on top of robot why do we have to convert points from robot
% coordinate frame to world coordinate ? 
% actually it converts robot coordinate frame -> world coordinate frame. 
% placing the scan in global context. 
% we need transformation because 
% 1. when you convert polar -> cartesian scan points are relative to robot.
% 2. robot is at (0,0). a wall straight ahead 5m is (5, 0)
% 3. but we are building a GLOBAL MAP
% 4. the robot is moving in world. to build a consistent map we must place
% each scan in a fixed world frame. this way features align over time. 
