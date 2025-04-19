clc;
clear;
close all;

% Create a 3D simulation scenario with update rate of 5 Hz
scenario = robotScenario(UpdateRate=5);

% Add the floor to the environment
floorColor = [0.5882 0.2941 0];
addMesh(scenario,"Plane",Position=[5 5 0],Size=[10 10],Color=floorColor);

% Parameters for walls
wallHeight = 1;
wallWidth = 0.25;
wallLength = 10;
wallColor = [1 1 0.8157];

% Add outer walls (creating a closed square environment)
addMesh(scenario,"Box",Position=[wallWidth/2, wallLength/2, wallHeight/2],...
    Size=[wallWidth, wallLength, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[wallLength-wallWidth/2, wallLength/2, wallHeight/2],...
    Size=[wallWidth, wallLength, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[wallLength/2, wallLength-wallWidth/2, wallHeight/2],...
    Size=[wallLength, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[wallLength/2, wallWidth/2, wallHeight/2],...
    Size=[wallLength, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true);

% Add internal obstacles/walls inside the room
addMesh(scenario,"Box",Position=[wallLength/8, wallLength/3, wallHeight/2],...
    Size=[wallLength/4, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[wallLength/4, wallLength/3, wallHeight/2],...
    Size=[wallWidth, wallLength/6,  wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[(wallLength-wallLength/4), wallLength/2, wallHeight/2],...
    Size=[wallLength/2, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[wallLength/2, wallLength/2, wallHeight/2],...
    Size=[wallWidth, wallLength/3, wallHeight],Color=wallColor,IsBinaryOccupied=true);

% Show the 3D environment
show3D(scenario);
lightangle(-45,30);
view(60,50);

% Generate a 2D binary occupancy map from the 3D scenario
map = binaryOccupancyMap(scenario,GridOriginInLocal=[-2 -2],...
    MapSize=[15 15],...
    MapHeightLimits=[0 3]);
inflate(map,0.3); % Add safety margin to obstacles
show(map)

% Define start and goal positions for the robot
startPosition = [1 1];
goalPosition = [8 8];

% Plan a path using PRM (Probabilistic Roadmap) planner
rng(100)
numnodes = 2000;
planner = mobileRobotPRM(map,numnodes);
planner.ConnectionDistance = 1;
waypoints = findpath(planner,startPosition,goalPosition);

% Create a trajectory based on the planned path
robotheight = 0.12;
numWaypoints = size(waypoints,1);
firstInTime = 0;
speedFactor = 3; % Higher value = slower robot speed
lastInTime = firstInTime + speedFactor*(numWaypoints-1);
timeOfArrival = linspace(firstInTime, lastInTime, numWaypoints);
traj = waypointTrajectory(SampleRate=10,...
    TimeOfArrival=timeOfArrival, ...
    Waypoints=[waypoints, robotheight*ones(numWaypoints,1)], ...
    ReferenceFrame="ENU");

% Load the Clearpath Husky robot model
huskyRobot = loadrobot("clearpathHusky");

% Add the robot to the scenario and assign the trajectory
platform = robotPlatform("husky",scenario, RigidBodyTree=huskyRobot,...
    BaseTrajectory=traj);

% Configure 3D lidar sensor
Lidar_Range = 10;
lidarModel = robotLidarPointCloudGenerator(...
    UpdateRate=100, ...
    MaxRange=5, ...
    RangeAccuracy=0.20, ...
    AzimuthResolution=0.16, ...
    ElevationResolution=1.25, ...
    AzimuthLimits=[-180 180], ...
    ElevationLimits=[0 10], ...
    HasNoise=false, ...
    HasOrganizedOutput=true);

% Mount the lidar sensor on the robot
lidar = robotSensor("lidar", platform, lidarModel, ...
    MountingLocation=[0 0 0.3], MountingAngles=[0 0 0],UpdateRate=scenario.UpdateRate);

% Initialize 2D Lidar SLAM
maxLidarRange = 10;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;
slamAlg.LoopClosureSearchRadius = 8;
slamAlg.OptimizationInterval = 10;

% Initialize occupancy map
occupancyMap = occupancyMap(20, 20, 20);

% Start the simulation
[ax,plotFrames] = show3D(scenario);
lightangle(-45,30)
view(60,50)
hold(ax,"on")
plot(ax,waypoints(:,1),waypoints(:,2),"-ms",...
    LineWidth=2,...
    MarkerSize=4,...
    MarkerEdgeColor="b",...
    MarkerFaceColor=[0.5 0.5 0.5]);
hold(ax,"off")

setup(scenario)

% Run simulation with controlled time step (20 Hz)
r = rateControl(20);
robotStartMoving = false;
idx = 1;

while advance(scenario)
    show3D(scenario,Parent=ax,FastUpdate=true);
    waitfor(r);
    updateSensors(scenario);

    % Read robot pose and lidar data
    currentPose = read(platform);
    [~, ~, sensorReadings(idx)] = read(lidar);
    idx = idx + 1;

    if ~any(isnan(currentPose))
        robotStartMoving = true;
    end
    if any(isnan(currentPose)) && robotStartMoving
        break; % Stop simulation when robot finishes its trajectory
    end
end

% Restart scenario for post-processing
restart(scenario)

% Feed collected scans to SLAM and visualize first loop closure
firstTimeLCDetected = false;
figure;
for i=1:length(sensorReadings)
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, pc2scan(sensorReadings(i)));
    if ~isScanAccepted
        continue;
    end
    if optimizationInfo.IsPerformed && ~firstTimeLCDetected
        show(slamAlg, 'Poses', 'off');
        hold on;
        show(slamAlg.PoseGraph);
        hold off;
        firstTimeLCDetected = true;
        drawnow
    end
end
title('First loop closure');

% Show full SLAM map and trajectory
figure
show(slamAlg);
title({'Final Built Map of the Environment', 'Trajectory of the Robot'});

% Build occupancy map from optimized poses and scans
[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

% Display the final occupancy grid map
figure;
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title('Occupancy Grid Map Built Using Lidar SLAM');
