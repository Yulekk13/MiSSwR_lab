clear 
clc

%Tworzenie siatki
map = binaryOccupancyMap(100,100,10 ,"grid");

%Tworzenie ścian
walls = zeros(100,100);
walls(1,:) = 1; % Górna ściana
walls(end,:) = 1; % Dolna ściana
walls(:,1) = 1; % Lewa ściana
walls(:,end) = 1; % Prawa ściana

%Ściany wewnętrzne
walls(1:40, 25) = 1;
walls(1:60,50) = 1;
walls(70,15:35) = 1; 
walls(60:100,75) = 1; 
walls(35,75:100) = 1; 
walls(90:100,50) = 1; 

setOccupancy(map,[1 1],walls,"grid")
% show(map)

% Set start and goal poses.
start = [10 10 0];
goal = [90 90 0];

% Show start and goal positions of robot.
hold on
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')

% Show start and goal heading angle using a line.
r = 0.5;
plot([start(1),start(1) + r*cos(start(3))],[start(2),start(2) + r*sin(start(3))],'r-')
plot([goal(1),goal(1) + r*cos(goal(3))],[goal(2),goal(2) + r*sin(goal(3))],'m-')
hold off

%%

inflatedMap = map;
inflate(inflatedMap,0.1);

ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

sv = validatorOccupancyMap(ss);
sv.Map = inflatedMap;
sv.ValidationDistance = 0.01;

planner = plannerPRM(ss,sv); % tutaj wybierasz funkcje  plannerPRM(ss,sv, "MaxConnectionDistance", 1, "MaxNumNodes", 200)

graph = graphData(planner);

edges = table2array(graph.Edges);
nodes = table2array(graph.Nodes);

start = [10 10 0];
goal = [90 90 0];

show(sv.Map)
hold on
plot(nodes(:,1),nodes(:,2),"*","Color","b","LineWidth",2)
for i = 1:size(edges,1)
    % Samples states at distance 0.02 meters.
    states = interpolate(ss,nodes(edges(i,1),:), ...
                         nodes(edges(i,2),:),0:0.02:1);
    plot(states(:,1),states(:,2),"Color","b")
end
plot(start(1),start(2),"*","Color","g","LineWidth",3)
plot(goal(1),goal(2),"*","Color","r","LineWidth",3)


rng(100,"twister");
[pthObj, solnInfo] = plan(planner,start,goal);

if solnInfo.IsPathFound
    interpolate(pthObj,1000);
    plot(pthObj.States(:,1),pthObj.States(:,2), ...
         "Color",[0.85 0.325 0.098],"LineWidth",2)
else
    disp("Path not found")
end
hold off