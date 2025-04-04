clc
close all
clear

%% User input for parameters
maxConnectionDistance = input('Enter MaxConnectionDistance: ');
maxIterations = input('Enter MaxIterations: ');
validationDistance = input('Enter ValidationDistance: ');

%%
tic
map = binaryOccupancyMap(100, 100, 1);
occ = zeros(100, 100);

occ(1,:) = 1;       % Top wall
occ(end,:) = 1;     % Bottom wall
occ(:,1) = 1;  % Left wall (with a gap in the middle)
occ(:,end) = 1; % Right wall (with a gap in the middle)

% Block Fill
% First map
occ(1:40, 25) = 1;
occ(1:60, 50) = 1;
occ(70, 15:35) = 1;
occ(60:100, 75) = 1;
occ(35, 75:100) = 1;
occ(90:100, 50) = 1;

% Second map
% occ(20, 1:20) = 1;
% occ(1:20, 55) = 1;
% occ(20:40, 40) = 1;
% occ(20:40, 75) = 1;
% occ(40:60, 80) = 1;
% occ(40, 20:85) = 1;
% occ(40:55, 20) = 1;
% occ(60:80, 35) = 1;
% occ(80:90, 50) = 1;
% occ(60:80, 65) = 1;
% occ(80, 20:65) = 1;
% occ(80, 80:100) = 1;


% Outer walls
occ(1, :) = 1; % Top wall
occ(end, :) = 1; % Bottom wall
occ(:, 1) = 1; % Left wall
occ(:, end) = 1; % Right wall

setOccupancy(map, occ);

figure
show(map)
title('Custom Floor Plan')

%%
inflatedMap = map;
inflate(inflatedMap,0.1);

% Set start and goal poses
start = [10,90,0];
goal = [90,10,0];

% Show start and goal positions of robot
hold on
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')

% Show start and goal heading angle using a line
r = 0.5;
plot([start(1),start(1) + r*cos(start(3))],[start(2),start(2) + r*sin(start(3))],'r-')
plot([goal(1),goal(1) + r*cos(goal(3))],[goal(2),goal(2) + r*sin(goal(3))],'m-')
hold off

bounds = [inflatedMap.XWorldLimits; inflatedMap.YWorldLimits; [-pi pi]];

ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.4;

stateValidator = validatorOccupancyMap(ss);
stateValidator.Map = inflatedMap;
stateValidator.ValidationDistance = validationDistance;

planner = plannerRRT(ss,stateValidator);
planner.MaxConnectionDistance = maxConnectionDistance;
planner.MaxIterations = maxIterations;

planner.GoalReachedFcn = @exampleHelperCheckIfGoal;

rng default
[pthObj,solnInfo] = plan(planner,start,goal);

if isempty(pthObj.States)
    error('Planner did not find a path! Check the map, start, goal, and planner parameters.')
end

shortenedPath = shortenpath(pthObj,stateValidator)
originalLength = pathLength(pthObj)
shortenedLength = pathLength(shortenedPath)

toc

show(map)
titleText = sprintf('Algorithm RRT\nMaxConnectionDistance: %.2f | MaxIterations: %d | ValidationDistance: %.2f', ...
    maxConnectionDistance, maxIterations, validationDistance);
title(titleText);
hold on

% Plot entire search tree.
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),plannerLineSpec.tree{:})

% Interpolate and plot path.
interpolate(pthObj,500)
plot(pthObj.States(:,1),pthObj.States(:,2),plannerLineSpec.path{:})

% Interpolate and plot path.
interpolate(shortenedPath,500);
plot(shortenedPath.States(:,1),shortenedPath.States(:,2),'g-','LineWidth',3)

% Show start and goal in grid map.
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')
legend('search tree','original path','shortened path')

hold off
%% Goal-reaching function
function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
isReached = false;
threshold = 0.1;
if planner.StateSpace.distance(newState, goalState) < threshold
    isReached = true;
end
end
