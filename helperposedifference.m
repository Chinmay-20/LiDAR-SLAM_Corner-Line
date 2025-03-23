function dp = helperposedifference(pose1, pose2)
    
    % disp(pose1);
    % disp(pose2);
    dp = pose2 - pose1;
    % disp(dp);
    dp(3) = angdiff(pose1(3), pose2(3));
    % disp(dp(3));
    
end

% computes difference between two poses
% current the angle to ensure it stays within ( -pie, pie) using anglediff