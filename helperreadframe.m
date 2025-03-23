% Read a laser scan
function scan = helperreadframe(lidar_data, idx, lidar, usableRange)

    
angles = lidar.angles;
ranges = lidar_data.ranges(idx, :)';

% Remove points whose range is not so trustworthy
maxRange = min(lidar.range_max, usableRange);
isBad = (ranges < lidar.range_min) | (ranges > maxRange);
angles(isBad) = [];
ranges(isBad) = [];

% Convert from polar coordinates to cartesian coordinates
[xs, ys] = pol2cart(angles, ranges);
scan = [xs, ys]; 
end

% preprocessing is done on input data, from lidar frame we get a distances of object ranging from 0 to infinity. 
% to avoid distant noise, focus on local mapping increase SLAM robustness. 
% i have specified range. in this if value is not in range we ignore that 
% result in some frames data reduced from 1079 to 1051. 
% for same distances ignored, we also ignore angle values.

% polar to cartesian coordinates is also part of preprocessing as we track robot in 2D plane. 