function helperplottingmap(cfig, map, path, scan, scanIdx)

world  = map.points;
scan = helpertransformrobottoworldcoordinate(scan, path(:,end));

worldColor = [0.5 0.5 1]; 
scanColor = 'blue';

% Plot
cfig(1); clf; 
set(gca, 'color', [0,0,0]);
hold on; grid on; axis equal;

plot(world(:,1), world(:,2), '+', 'MarkerSize', 0.1, 'color', worldColor);
plot(scan(:,1),  scan(:,2),  '.', 'MarkerSize', 2, 'color', scanColor);
plot(path(1,:),  path(2,:),  '-.', 'LineWidth', 1, 'color', [0.5 1 0.5]); 

for i = 1:20:length(scan)
    line([path(1,end), scan(i,1)], [path(2,end), scan(i,2)], 'color', [0.75 0.996 0.242]);
end
title(['Scan: ', num2str(scanIdx)]);
drawnow


