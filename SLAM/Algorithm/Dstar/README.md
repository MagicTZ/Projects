
# Geolocalization and Vehicle Navigation :robot: 

###### Tags: `Algorithm` `Project` `Slam` `Matlab`

## Exercise 01 -- Pathfinding (D* algorithm) :thinking:


### 1 Principles
#### 1.1 How does it work?

Firstly, D* algorithm is an algorithm to calculate and update the list of nodes in order to find the most proper way in a navigation process. It had been raised by Anthony Stentz in 1994, and the name came from Dynamic A* algorithm. The nodes list, called as open list, contains the nodes, which are in several different states. The states and the meanings of them are shown below:

* NEW: It means that this node has never been included in the open list

* OPEN: It means this node is currently in the open list

* CLOSED: It means this node is not in the open list

* RAISE: It means that its cost is higher than last time being in open list

* LOWER: It means that its cost is lower than last time being in open list



Secondly, the algorithm includes expansion and obstacles handling.

The algorithm works by iteratively selecting a node from the open list and evaluating it. Then, it propagates the changes of the node to all adjacent nodes and puts them in the open list. This propagation process is called ‘expansion’. Unlike canonical A* algorithm, which follows the path from start to finish, D * searches backward from the target node. Each expansion node has a reverse pointer, which points to the next node that points to the target, and each node knows the exact cost of the target. When the starting node is the next node to be expanded, the algorithm is completed, and the path of the target can be found by following the reverse pointer.

When an obstacle is detected on the specified path, all affected points will be placed in the open list again, this time marked RAISE. However, before a RAISED node increases costs, the algorithm checks its neighbors and checks whether it can reduce the cost of the node. If not, the promotion state is propagated to the descendants of all nodes, that is, nodes with reverse pointers. Then the algorithm will evaluate these nodes and pass the RAISE status to form a wave. When the RAISED node can be reduced, its reverse pointer will be updated and pass the LOWER state to its neighbors. These RAISE and LOWER state waves are the core of D *. By this time, a series of other points will not be "touched" by the waves. Therefore, the algorithm is only applicable to points affected by cost changes.

#### 1.2 Pros and Cons

* __Advantages__

D * algorithm is very effective in pathfinding especially in dynamic environments because it reduce the redundant calculations of the same data.

* __Disadvantages__

While dealing changes that occur on the processing of finding the shortest path at a long distance, it will not be applicable.



### 2 Questions and Answers regarding the principle

___Question 1:___ 

When a new detected obstacle is found during the movement, which process will be taken immediately?

___Answer:___

A.   This path is invalidated and calculated again from the goal point

___B.  Use the Modify-Cost function immediately to correct the path cost and put the affected state back in the open list___

C.   Assuming that the obstacle does not exist, adjust after crossing the obstacle

D.   Calculate the new path again from the start point when starting from the goal point can not work.

---
___Question 2:___

What is the most prominent difference between A* algorithm and D* algorithm?

___Answer:___

A.  A* algorithm searches backward from the target node; D* algorithm follows the path from start to finish. 
___B.  A* algorithm follows the path from start to finish; D* algorithm searches backward from the target node.___
C. They both follow the path from start to finish. 
D. They both search backward from the target node. 

---
___Question 3:___

In D star algorithm, we always What is the end condition (finish condition) of the D star algorithm?

___Answer:___

A. Goal State's condition is closed.

B. The cost function is lowest.

___C. OpenList is empty or the tag of the start point is closed.___

D. OpenList is empty and the tag of the goal state is closed.

------



### 3 Implement the algorithm in Matlab

By using Robotics Toolbox, we take advantage of the Dstar navigation class, which is a subclass of the abstract Navigation class. 

In our own path-finding function, we first transform the binary occupancy map back into a matrix to fit the parameter’s data type requirement of the Dstar function. Because of this transformation, the difference in map scale is quite vital. Then we time the path-finding computation, and get the list of waypoints result. The length of path is calculated after we call the function. 

```cpp=
//write a function
[waypoints] = findpath(map,start,end)
//input: binary occupancy map, start and end point as [x y] pairs
//output: list of waypoints as [x y] pairs
```

```matlab
clear;
clc;

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

ds = Dstar(mat);    % create navigation object
tic; ds.plan(goal); toc;      % create plan for specified goal and time the computation
ds.path(start);     % animate path from this start location
dstar = ds.path(start) / 20;   % get waypoints

end
```

https://blog.csdn.net/a380331382/article/details/82841071

Occupancy grid: https://de.mathworks.com/help/robotics/ug/occupancy-grids.html



### 4 Results

#### 4.1 Requirements

* Create a plot of the calculated path
* Evaluate computing time and path length

<img src="https://i.imgur.com/ONVQYY0.png" width="50%" height="350">
<center> Figure1 The inflated image</center>

<img src="https://i.imgur.com/MXxgqHc.png" width="50%" height="350">
<center>Figure2 Final path in Binary Occupancy Grid</center>


| start [m]  |  end [m]   | Timing [s] | The length of the path [m] |
| :--------: | :--------: | :--------: | :------------------------: |
| (0.2, 3.7) | (3.8, 0.5) |  1.193856  |            7.5             |



### 5 Tools and Literatures
* Robotics System Toolbox (Matlab)
* Robotics Vision and Control Toolbox (Peter Corke)

* Matlab blog about PathFinding: https://blogs.mathworks.com/steve/2011/12/21/exploring-shortest-paths-wrapping-up
* Example of Pathfinding using the "Robotics System Toolbox: https://de.mathworks.com/help/robotics/examples/path-planning-in-environments-of-difference-complexity.html;jsessionid=b3c6e35154d35d444acb5b0dcb53

