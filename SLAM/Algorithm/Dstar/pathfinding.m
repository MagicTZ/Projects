clear;
clc;
close all;

%% Read the BMP Image and Transform It Into BinaryOccupancyMap
img = 1 - imread('TestMap.bmp','bmp');  
map = binaryOccupancyMap(img,20);  % resolution: 5cm
% show(map); 

%% Define Start Point and End Point
start = [0.2,3.7]; 
goal = [3.8,0.5]; 

%% Define Robot Dimensions and Inflate the Map
robotRadius = 0.2; % meters
mapInflated = copy(map);
inflate(mapInflated,robotRadius)
% show(mapInflated)

%% DStar Method Combined With Navigation Toolbox
waypoints = findpath(mapInflated, start, goal);
% disp(waypoints);
fprintf('The length of the path is %.2f\n', size(waypoints,1)/20);

%% Plot the Way Points on the Map
setOccupancy(map,waypoints,1)
figure
show(map);

%% DStar
function dstar = findpath(map, start, goal)
mat = occupancyMatrix(map);
mat = flipud(mat + 0);
% imshow(mat);

start = start * 20;
goal = goal * 20;

tic;
ds = Dstar(mat);    % create navigation object
ds.plan(goal);toc; % create plan for specified goal and time the computation
ds.path(start);     % animate path from this start location
dstar = ds.path(start) / 20;   % get waypoints

end


