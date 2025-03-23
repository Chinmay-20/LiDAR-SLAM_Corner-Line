% takes in line segments (defined by start and end points) 
% and returns densely interpolated points along each line 

% endpoint = [ x1 y1 x2 y2 x3 y3 ....;
%              x1' y1' x2' y2' x3' y3'...] 
% line 1 from x1, y1 to x1' y1'
% line 2 from x2 y2 to x2'y2'

function point = helperexpandlinepointswithlinefeature(endPoint)
% Inuput
%   line:slope and central points
%   map:first and end points
% Output
%   point:expanded line points with line feature

if isempty(endPoint)
    point = [];
    return
end

% since each line uses 2 columns, total number of lines = half the column
% count
k = size(endPoint,2)/2;


point = zeros(1,2);
% keeps track of where the next block of points should be inserted in
% point array
front = 0;

% loping through all k lines
for i = 1 : k
    % compute line length in meters using euclidean distance
    % round up to nearest integer
    % creates 100 points per 1 meter of line, dynamically decides how many
    % points to interpolate based on line length. 
    length = 100*ceil(norm([endPoint(1,2*i-1) endPoint(1,2*i)]-[endPoint(2,2*i-1) endPoint(2,2*i)]));
    
    % linespace generates N evenly spaced values from a to b, filling in
    % all x and y coordinates along line.
    point(front + 1:front + length,1) = linspace(endPoint(1,2*i-1),endPoint(2,2*i-1),length)';
    point(front + 1:front + length,2) = linspace(endPoint(1,2*i),endPoint(2,2*i),length)';
    front = size(point,1); % update insertion index
end

return