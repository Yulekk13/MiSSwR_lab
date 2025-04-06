baseDownloadURL = 'https://www.mrt.kit.edu/z/publ/download/velodyneslam/data/scenario1.zip';
dataFolder = fullfile(tempdir,'kit_velodyneslam_data_scenario1',filesep);
options = weboptions('Timeout',Inf);

zipFileName  = dataFolder + "scenario1.zip";

% Get the full file path to the PNG files in the scenario1 folder
pointCloudFilePattern = fullfile(dataFolder,'scenario1','scan*.png');
numExpectedFiles = 2513;

folderExists = exist(dataFolder,'dir');
if ~folderExists
    % Create a folder in a temporary directory to save the downloaded zip
    % file
    mkdir(dataFolder);

    disp('Downloading scenario1.zip (153 MB) ...')
    websave(zipFileName,baseDownloadURL,options);

    % Unzip downloaded file
    unzip(zipFileName,dataFolder);

elseif folderExists && numel(dir(pointCloudFilePattern)) < numExpectedFiles
    % Redownload the data if it got reduced in the temporary directory
    disp('Downloading scenario1.zip (153 MB) ...')
    websave(zipFileName,baseDownloadURL,options);

    % Unzip downloaded file
    unzip(zipFileName,dataFolder)
end

datasetTable = helperReadDataset(dataFolder,pointCloudFilePattern);

pointCloudTable = datasetTable(:,1);
insDataTable = datasetTable(:,2:end);

ptCloud = helperReadPointCloudFromFile(pointCloudTable.PointCloudFileName{1});
disp(ptCloud)

disp(insDataTable(1,:))

% Specify limits of the player
xlimits = [-45 45]; % meters
ylimits = [-45 45];
zlimits = [-10 20];

% Create a streaming point cloud display object
lidarPlayer = pcplayer(xlimits,ylimits,zlimits);

% Customize player axes labels
xlabel(lidarPlayer.Axes,'X (m)')
ylabel(lidarPlayer.Axes,'Y (m)')
zlabel(lidarPlayer.Axes,'Z (m)')

title(lidarPlayer.Axes,'Lidar Sensor Data')

% Skip evey other frame since this is a long sequence
skipFrames = 2;
numFrames = height(pointCloudTable);
for n = 1:skipFrames:numFrames

    % Read a point cloud
    fileName = pointCloudTable.PointCloudFileName{n};
    ptCloud = helperReadPointCloudFromFile(fileName);

    % Visualize point cloud
    view(lidarPlayer,ptCloud);

    pause(0.01)
end

hide(lidarPlayer)

% Set random seed to ensure reproducibility
rng(0);

% Create an empty view set
vSet = pcviewset;

% Initialize point cloud processing parameters
downsamplePercent = 0.1;
regGridSize = 3;

% Initialize transformations
absTform = rigidtform3d;  % Absolute transformation to reference frame
relTform = rigidtform3d;  % Relative transformation between successive scans

viewId = 1;
skipFrames = 5;
numFrames = height(pointCloudTable);
displayRate = 100;  % Update display every 100 frames
for n = 1:skipFrames:numFrames

    % Read point cloud
    fileName = pointCloudTable.PointCloudFileName{n};
    ptCloudOrig = helperReadPointCloudFromFile(fileName);

    % Process point cloud
    %   - Segment and remove ground plane
    %   - Segment and remove ego vehicle
    ptCloud = helperProcessPointCloud(ptCloudOrig);

    % Downsample the processed point cloud
    ptCloud = pcdownsample(ptCloud,"random",downsamplePercent);

    firstFrame = (n==1);
    if firstFrame
        % Add first point cloud scan as a view to the view set
        vSet = addView(vSet,viewId,absTform,"PointCloud",ptCloudOrig);

        viewId = viewId + 1;
        ptCloudPrev = ptCloud;
        continue;
    end

    % Use INS to estimate an initial transformation for registration
    initTform = helperComputeInitialEstimateFromINS(relTform, ...
        insDataTable(n-skipFrames:n,:));

    % Compute rigid transformation that registers current point cloud with
    % previous point cloud
    relTform = pcregisterndt(ptCloud,ptCloudPrev,regGridSize, ...
        "InitialTransform",initTform);

    % Update absolute transformation to reference frame (first point cloud)
    absTform = rigidtform3d(absTform.A * relTform.A);

    % Add current point cloud scan as a view to the view set
    vSet = addView(vSet,viewId,absTform,"PointCloud",ptCloudOrig);

    % Add a connection from the previous view to the current view, representing
    % the relative transformation between them
    vSet = addConnection(vSet,viewId-1,viewId,relTform);

    viewId = viewId + 1;

    if mod(viewId,displayRate) == 0
        plot(vSet)
        drawnow
    end

    ptCloudPrev = ptCloud;
    initTform = relTform;
end

title('View Set Display')

% Display the first few views and connections
head(vSet.Views)

head(vSet.Connections)

ptClouds = vSet.Views.PointCloud;
absPoses = vSet.Views.AbsolutePose;
mapGridSize = 0.2;
ptCloudMap = pcalign(ptClouds,absPoses,mapGridSize);

figure
pcshow(ptCloudMap);
hold on
plot(vSet)
title('Point Cloud Map','Color','w')

% Set random seed to ensure reproducibility
rng(0);

% Create an empty view set
vSet = pcviewset;

% Create a loop closure detector
loopDetector = scanContextLoopDetector;

% Initialize transformations
absTform = rigidtform3d;  % Absolute transformation to reference frame
relTform = rigidtform3d;  % Relative transformation between successive scans

maxTolerableRMSE = 3;  % Maximum allowed RMSE for a loop closure candidate to be accepted

viewId = 1;
for n = 1:skipFrames:numFrames

    % Read point cloud
    fileName = pointCloudTable.PointCloudFileName{n};
    ptCloudOrig = helperReadPointCloudFromFile(fileName);

    % Process point cloud
    %   - Segment and remove ground plane
    %   - Segment and remove ego vehicle
    ptCloud = helperProcessPointCloud(ptCloudOrig);

    % Downsample the processed point cloud
    ptCloud = pcdownsample(ptCloud,"random",downsamplePercent);

    firstFrame = (n==1);
    if firstFrame
        % Add first point cloud scan as a view to the view set
        vSet = addView(vSet,viewId,absTform,"PointCloud",ptCloudOrig);

        % Extract the scan context descriptor from the first point cloud
        descriptor = scanContextDescriptor(ptCloudOrig);

        % Add the first descriptor to the loop closure detector
        addDescriptor(loopDetector,viewId,descriptor)

        viewId = viewId + 1;
        ptCloudPrev = ptCloud;
        continue;
    end

    % Use INS to estimate an initial transformation for registration
    initTform = helperComputeInitialEstimateFromINS(relTform, ...
        insDataTable(n-skipFrames:n,:));

    % Compute rigid transformation that registers current point cloud with
    % previous point cloud
    relTform = pcregisterndt(ptCloud,ptCloudPrev,regGridSize, ...
        "InitialTransform",initTform);

    % Update absolute transformation to reference frame (first point cloud)
    absTform = rigidtform3d(absTform.A * relTform.A);

    % Add current point cloud scan as a view to the view set
    vSet = addView(vSet,viewId,absTform,"PointCloud",ptCloudOrig);

    % Add a connection from the previous view to the current view representing
    % the relative transformation between them
    vSet = addConnection(vSet,viewId-1,viewId,relTform);

    % Extract the scan context descriptor from the point cloud
    descriptor = scanContextDescriptor(ptCloudOrig);

    % Add the descriptor to the loop closure detector
    addDescriptor(loopDetector,viewId,descriptor)

    % Detect loop closure candidates
    loopViewId = detectLoop(loopDetector);

    % A loop candidate was found
    if ~isempty(loopViewId)
        loopViewId = loopViewId(1);

        % Retrieve point cloud from view set
        loopView = findView(vSet,loopViewId);
        ptCloudOrig = loopView.PointCloud;

        % Process point cloud
        ptCloudOld = helperProcessPointCloud(ptCloudOrig);

        % Downsample point cloud
        ptCloudOld = pcdownsample(ptCloudOld,"random",downsamplePercent);

        % Use registration to estimate the relative pose
        [relTform,~,rmse] = pcregisterndt(ptCloud,ptCloudOld, ...
            regGridSize,"MaxIterations",50);

        acceptLoopClosure = rmse <= maxTolerableRMSE;
        if acceptLoopClosure
            % For simplicity, use a constant, small information matrix for
            % loop closure edges
            infoMat = 0.01 * eye(6);

            % Add a connection corresponding to a loop closure
            vSet = addConnection(vSet,loopViewId,viewId,relTform,infoMat);
        end
    end

    viewId = viewId + 1;

    ptCloudPrev = ptCloud;
    initTform = relTform;
end

G = createPoseGraph(vSet);
disp(G)

% Display view set to display loop closure detections
figure
hG = plot(vSet);

% Update axes limits to focus on loop closure detections
xlim([-50 50]);
ylim([-100 50]);

% Find and highlight loop closure connections
loopEdgeIds = find(abs(diff(G.Edges.EndNodes,1,2)) > 1);
highlight(hG,'Edges',loopEdgeIds,'EdgeColor','red','LineWidth',3)
title('Loop Closure Connections')

optimG = optimizePoseGraph(G,'g2o-levenberg-marquardt');

vSetOptim = updateView(vSet,optimG.Nodes);

figure
plot(vSetOptim)
title('View Set Display (after optimization)')

mapGridSize = 0.2;
ptClouds = vSetOptim.Views.PointCloud;
absPoses = vSetOptim.Views.AbsolutePose;
ptCloudMap = pcalign(ptClouds,absPoses,mapGridSize);

figure
pcshow(ptCloudMap);

% Overlay view set display
hold on
plot(vSetOptim);
title('Point Cloud Map (after optimization)','Color','w')

function datasetTable = helperReadDataset(dataFolder,pointCloudFilePattern)
%helperReadDataset Read Velodyne SLAM Dataset data into a timetable
%   datasetTable = helperReadDataset(dataFolder) reads data from the
%   folder specified in dataFolder into a timetable. The function
%   expects data from the Velodyne SLAM Dataset.
%
%   See also fileDatastore, helperReadINSConfigFile.

% Create a file datastore to read in files in the right order
fileDS = fileDatastore(pointCloudFilePattern,'ReadFcn', ...
    @helperReadPointCloudFromFile);

% Extract the file list from the datastore
pointCloudFiles = fileDS.Files;

imuConfigFile = fullfile(dataFolder,'scenario1','imu.cfg');
insDataTable = helperReadINSConfigFile(imuConfigFile);

% Delete the bad row from the INS config file
insDataTable(1447,:) = [];

% Remove columns that will not be used
datasetTable = removevars(insDataTable, ...
    {'Num_Satellites','Latitude','Longitude','Altitude','Omega_Heading', ...
    'Omega_Pitch','Omega_Roll','V_X','V_Y','V_ZDown'});

datasetTable = addvars(datasetTable,pointCloudFiles,'Before',1, ...
    'NewVariableNames',"PointCloudFileName");
end

function ptCloud = helperProcessPointCloud(ptCloudIn,method)
%helperProcessPointCloud Process pointCloud to remove ground and ego vehicle
%   ptCloud = helperProcessPointCloud(ptCloudIn,method) processes
%   ptCloudIn by removing the ground plane and the ego vehicle.
%   method can be "planefit" or "rangefloodfill".
%
%   See also pcfitplane, pointCloud/findPointsInCylinder.

arguments
    ptCloudIn (1,1) pointCloud
    method          string      {mustBeMember(method,["planefit","rangefloodfill"])} = "rangefloodfill"
end

isOrganized = ~ismatrix(ptCloudIn.Location);

if (method=="rangefloodfill" && isOrganized)
    % Segment ground using floodfill on range image
    groundFixedIdx = segmentGroundFromLidarData(ptCloudIn, ...
        "ElevationAngleDelta",11);
else
    % Segment ground as the dominant plane with reference normal
    % vector pointing in positive z-direction
    maxDistance = 0.4;
    maxAngularDistance = 5;
    referenceVector= [0 0 1];

    [~,groundFixedIdx] = pcfitplane(ptCloudIn,maxDistance, ...
        referenceVector,maxAngularDistance);
end

if isOrganized
    groundFixed = false(size(ptCloudIn.Location,1),size(ptCloudIn.Location,2));
else
    groundFixed = false(ptCloudIn.Count,1);
end
groundFixed(groundFixedIdx) = true;

% Segment ego vehicle as points within a cylindrical region of the sensor
sensorLocation = [0 0 0];
egoRadius = 3.5;
egoFixed = findPointsInCylinder(ptCloudIn,egoRadius,"Center",sensorLocation);

% Retain subset of point cloud without ground and ego vehicle
if isOrganized
    indices = ~groundFixed & ~egoFixed;
else
    indices = find(~groundFixed & ~egoFixed);
end

ptCloud = select(ptCloudIn,indices);
end

function initTform = helperComputeInitialEstimateFromINS(initTform,insData)

% If no INS readings are available, return
if isempty(insData)
    return;
end

% The INS readings are provided with X pointing to the front, Y to the left
% and Z up. Translation below accounts for transformation into the lidar
% frame.
insToLidarOffset = [0 -0.79 -1.73]; % See DATAFORMAT.txt
Tnow = [-insData.Y(end) insData.X(end) insData.Z(end)].' + insToLidarOffset';
Tbef = [-insData.Y(1)   insData.X(1)   insData.Z(1)].'   + insToLidarOffset';

% Since the vehicle is expected to move along the ground, changes in roll
% and pitch are minimal. Ignore changes in roll and pitch, use heading only.
Rnow = rotmat(quaternion([insData.Heading(end) 0 0],'euler','ZYX','point'),'point');
Rbef = rotmat(quaternion([insData.Heading(1)   0 0],'euler','ZYX','point'),'point');

T = [Rbef Tbef; 0 0 0 1] \ [Rnow Tnow; 0 0 0 1];

initTform = rigidtform3d(T);
end
