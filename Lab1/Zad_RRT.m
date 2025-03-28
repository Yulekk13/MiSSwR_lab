map = binaryOccupancyMap(100, 80, 1);
occ = zeros(80, 100);

occ(1,:) = 1;       % Górna ściana
occ(end,:) = 1;     % Dolna ściana
occ(:,1) = 1;  % Lewa ściana (z przerwą w środku)
occ(:,end) = 1; % Prawa ściana (z przerwą w środku)

%% Własne
%Blok Wypełnienie
% occ(y_start:y_start+height-1, x_start:x_start+width-1) = 1;
occ(20:20+40-1, 20:20+30-1) = 1;
occ(30:30+10-1, 50:50+15-1) = 1;
occ(20:20+30-1, 65:65+10-1) = 1;
occ(60:60+20-1, 65:65+10-1) = 1;
occ(60:60+10-1, 20:20+10-1) = 1;
occ(60:60+3-1, 90:90+3-1) = 1;
occ(50:50+3-1, 93:93+3-1) = 1;
occ(70:70+3-1, 93:93+3-1) = 1;
occ(70:70+3-1, 10:10+3-1) = 1;
occ(57:57+3-1, 13:13+3-1) = 1;

%Ściana pozioma
occ(30,75:end) = 1;
occ(50,65:85) = 1;
occ(20,75:90) = 1;
occ(10,20:75) = 1;
occ(40,10:20)=1;
occ(40,85:end)=1;
occ(50,1:10)=1;
occ(25,1:12)=1;
occ(70,45:65)=1;

%Ściana pionowa
occ(1:10,90)=1;
occ(50:70,85)=1;
occ(1:15,10)=1;
occ(1:10,20)=1;

%% Show
setOccupancy(map, occ)

figure
show(map)
% view(3)
title('Custom Floor Plan')

%% Path
inflatedMap = map;
inflate(inflatedMap,0.1);

% Set start and goal poses.
start = [80,55,0];
goal = [95,45,0];



% Show start and goal positions of robot.
hold on
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')

% Show start and goal heading angle using a line.
r = 0.5;
plot([start(1),start(1) + r*cos(start(3))],[start(2),start(2) + r*sin(start(3))],'r-')
plot([goal(1),goal(1) + r*cos(goal(3))],[goal(2),goal(2) + r*sin(goal(3))],'m-')
hold off



bounds = [inflatedMap.XWorldLimits; inflatedMap.YWorldLimits; [-pi pi]];

ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.4;


stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = inflatedMap;
stateValidator.ValidationDistance = 0.05;

planner = plannerRRT(ss,stateValidator);
planner.MaxConnectionDistance = 2.0;
planner.MaxIterations = 120000;

planner.GoalReachedFcn = @exampleHelperCheckIfGoal;


rng default
[pthObj,solnInfo] = plan(planner,start,goal);

if isempty(pthObj.States)
    error('Planner nie znalazł ścieżki! Sprawdź mapę, start, goal i parametry planera.')
end


shortenedPath = shortenpath(pthObj,stateValidator);
originalLength = pathLength(pthObj)
shortenedLength = pathLength(shortenedPath)

show(map)
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

%%
function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
    isReached = false;
    threshold = 0.1;
    if planner.StateSpace.distance(newState, goalState) < threshold
        isReached = true;
    end
end

