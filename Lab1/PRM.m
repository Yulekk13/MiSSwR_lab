clc
close all
clear

%% User input for parameters
maxConnectionDistance = input('Enter MaxConnectionDistance: ');
maxNodes = input('Enter MaxNodes: ');

%%

map = binaryOccupancyMap(100, 100, 1);
occ = zeros(100, 100);

occ(1,:) = 1;       % Upper wall
occ(end,:) = 1;     % Lower wall
occ(:,1) = 1;       % Left wall (with a gap in the middle)
occ(:,end) = 1;     % Right wall (with a gap in the middle)

% Block Fill
occ(1:40, 25) = 1;
occ(1:60, 50) = 1;
occ(70, 15:35) = 1;
occ(60:100, 75) = 1;
occ(35, 75:100) = 1;
occ(90:100, 50) = 1;

% External walls
occ(1, :) = 1;  % Upper wall
occ(end, :) = 1; % Lower wall
occ(:, 1) = 1;  % Left wall
occ(:, end) = 1; % Right wall

setOccupancy(map, occ);

%%
% Create a state space and update the state space bounds to be the same as the map limits
ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

% Create a state validator with stateSpaceSE2 using the map and set the validation distance
sv = validatorOccupancyMap(ss, Map=map);
sv.ValidationDistance = 0.01;

% Create a plannerPRM object
planner = plannerPRM(ss, sv, "MaxConnectionDistance", maxConnectionDistance, "MaxNumNodes", maxNodes);
% planner = plannerPRM(ss, sv);

% Retrieve graph as a digraph object
graph = graphData(planner);

% Extract nodes and edges from graph
edges = table2array(graph.Edges);
nodes = table2array(graph.Nodes);

% Specify the start and goal states
start = [10, 90, 0];
goal = [90, 10, 0];

% Plot map and graph
show(sv.Map)
titleText = sprintf('Algorithm PRM\nMaxConnectionDistance: %.2f | MaxNodes: %d', ...
    maxConnectionDistance, maxNodes);
title(titleText);
hold on
plot(nodes(:, 1), nodes(:, 2), "*", "Color", "b", "LineWidth", 2)
for i = 1:size(edges, 1)
    % Sample states at a distance of 0.02 meters
    states = interpolate(ss, nodes(edges(i, 1), :), nodes(edges(i, 2), :), 0:0.02:1);
    plot(states(:, 1), states(:, 2), "Color", "b")
end
plot(start(1), start(2), "*", "Color", "g", "LineWidth", 3)
plot(goal(1), goal(2), "*", "Color", "r", "LineWidth", 3)

% Plan a path with default settings. Set the rng seed for repeatability
rng(100, "twister");
[pthObj, solnInfo] = plan(planner, start, goal);

% Visualize the results
if solnInfo.IsPathFound
    interpolate(pthObj, 1000);
    plot(pthObj.States(:, 1), pthObj.States(:, 2), "Color", [0.85 0.325 0.098], "LineWidth", 2)
else
    disp("Path not found")
end
hold off
