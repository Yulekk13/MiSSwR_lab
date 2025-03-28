dataFolder = fullfile(tempdir,'DALES');
trainDataFolder = fullfile(dataFolder,'dales_las','train');
testDataFolder = fullfile(dataFolder,'dales_las','test');

%%

lasReader = lasFileReader(fullfile(trainDataFolder,'5080_54435.las'));
[pc,attr] = readPointCloud(lasReader,'Attributes','Classification');
labels = attr.Classification;

% Select only labeled data.
pc = select(pc,labels~=0);
labels = labels(labels~=0);
classNames = [
    "ground"
    "vegetation"
    "cars"
    "trucks"
    "powerlines"
    "fences"
    "poles"
    "buildings"
    ];
figure;
ax = pcshow(pc.Location,labels);
helperLabelColorbar(ax,classNames);
title("Point Cloud with Overlaid Semantic Labels");

%%
blocksize = [51 51 Inf];

fs = matlab.io.datastore.FileSet(trainDataFolder);

bpc = blockedPointCloud(fs,blocksize);

numClasses = numel(classNames);
[weights,maxLabel,maxWeight] = helperCalculateClassWeights(fs,numClasses);

%%
ldsTrain = blockedPointCloudDatastore(bpc,MinPoints=10);

labelIDs = 1 : numClasses;

ptcld = preview(ldsTrain);
figure;
pcshow(ptcld.Location);
title("Cropped Point Cloud");

numPoints = 8192;

ldsTransformed = transform(ldsTrain,@(x,info) helperTransformToTrainData(x, ...
    numPoints,info,labelIDs,classNames),'IncludeInfo',true);
read(ldsTransformed)

%%
net = pointnetplusNetwork(numPoints,3,numClasses);

% Define the loss function.
lossfun = @(Y,T) mean(mean(sum(crossentropy(Y,T,weights,'WeightsFormat','UC','Reduction','none'),3),1),4);

%%
executionEnvironment = "auto";
if canUseParallelPool
    dispatchInBackground = true;
else
    dispatchInBackground = false;
end

learningRate = 0.0005;
l2Regularization = 0.01;
numEpochs = 20;
miniBatchSize = 16;
learnRateDropFactor = 0.1;
learnRateDropPeriod = 10;
gradientDecayFactor = 0.9;
squaredGradientDecayFactor = 0.999;

options = trainingOptions('adam', ...
    'InitialLearnRate',learningRate, ...
    'L2Regularization',l2Regularization, ...
    'MaxEpochs',numEpochs, ...
    'MiniBatchSize',miniBatchSize, ...
    'LearnRateSchedule','piecewise', ...
    'LearnRateDropFactor',learnRateDropFactor, ...
    'LearnRateDropPeriod',learnRateDropPeriod, ...
    'GradientDecayFactor',gradientDecayFactor, ...
    'SquaredGradientDecayFactor',squaredGradientDecayFactor, ...
    'ExecutionEnvironment',executionEnvironment, ...
    'DispatchInBackground',dispatchInBackground, ...
    'Plots','training-progress');

%%
doTraining = false;
if doTraining
    % Train the network on the ldsTransformed datastore using 
    % the trainnet function.
    [net,info] = trainnet(ldsTransformed,net,lossfun,options);
else
    % Load the pretrained network.
    load('pointnetplusNetworkTrained','net');
end

%%
tbpc = blockedPointCloud(fullfile(testDataFolder,'5080_54470.las'),blocksize);
tbpcds = blockedPointCloudDatastore(tbpc);

numNearestNeighbors = 20;
radius = 0.05;

labelsDensePred = [];

while hasdata(tbpcds)

    % Read the block along with block information.
    ptCloudDense = read(tbpcds);

    % Use the helperDownsamplePoints function, attached to this example as a
    % supporting file, to extract a downsampled point cloud from the
    % dense point cloud.
    ptCloudSparse = helperDownsamplePoints(ptCloudDense{1},[],numPoints);
                       
    % Use the helperNormalizePointCloud function, attached to this example as
    % a supporting file, to normalize the point cloud between 0 and 1.
    ptCloudSparseNormalized = helperNormalizePointCloud(ptCloudSparse);
    ptCloudDenseNormalized = helperNormalizePointCloud(ptCloudDense{1});
    
    % Use the helperTransformToTestData function, defined at the end of this
    % example, to convert the point cloud to a cell array and to permute the
    % dimensions of the point cloud to make it compatible with the input layer
    % of the network.
    ptCloudSparseForPrediction = helperTransformToTestData(ptCloudSparseNormalized);
    
    % Get the output predictions.
    labelsSparsePred = predict(net,ptCloudSparseForPrediction{1,1});
    [~,labelsSparsePred] = max(labelsSparsePred,[],3);
    
    % Use the helperInterpolate function, attached to this example as a
    % supporting file, to calculate labels for the dense point cloud,
    % using the sparse point cloud and labels predicted on the sparse point cloud.
    interpolatedLabels = helperInterpolate(ptCloudDenseNormalized, ...
        ptCloudSparseNormalized,labelsSparsePred,numNearestNeighbors, ...
        radius,maxLabel,numClasses);  
    
    % Concatenate the predicted labels from the blocks.
    labelsDensePred = vertcat(labelsDensePred,interpolatedLabels);
end

figure;
ax = pcshow(ptCloudDense{1}.Location,interpolatedLabels);
axis off;
helperLabelColorbar(ax,classNames);
title("Point Cloud Overlaid with Detected Semantic Labels");

%%
labelsDenseTarget = [];

reset(tbpcds);

while hasdata(tbpcds)    
    % Read the block along with block information.
    [~,infoDense] = read(tbpcds);

    % Extract the labels from the block information.
    labelsDense = infoDense.PointAttributes.Classification;
    
    % Concatenate the target labels from the blocks.
    labelsDenseTarget = vertcat(labelsDenseTarget,labelsDense);
end

confusionMatrix = segmentationConfusionMatrix(double(labelsDensePred), ...
    double(labelsDenseTarget),'Classes',1:numClasses);
metrics = evaluateSemanticSegmentation({confusionMatrix},classNames,'Verbose',false);

metrics.DataSetMetrics

metrics.ClassMetrics

%%
function helperLabelColorbar(ax,classNames)
% Colormap for the original classes.
cmap = [[0 0 255];
    [0 255 0];
    [255 192 203];
    [255 255 0];
    [255 0 255];
    [255 165 0];
    [139 0 150];
    [255 0 0]];
cmap = cmap./255;
cmap = cmap(1:numel(classNames),:);
colormap(ax,cmap);

% Add colorbar to current figure.
c = colorbar(ax);
c.Color = 'w';

% Center tick labels and use class names for tick marks.
numClasses = size(classNames,1);
c.Ticks = 1:1:numClasses;
c.TickLabels = classNames;

% Remove tick mark.
c.TickLength = 0;
end

%%
function [cellout,dataout] = helperTransformToTrainData(data,numPoints,info,...
    labelIDs,classNames)
if ~iscell(data)
    data = {data};
end
numObservations = size(data,1);
cellout = cell(numObservations,2);
dataout = cell(numObservations,2);
for i = 1:numObservations 
    classification = info.PointAttributes(i).Classification;

    % Remove labels with zero value.
    pc = data{i,1};
    pc = select(pc,(classification ~= 0));
    classification = classification(classification ~= 0);

    % Use the helperDownsamplePoints function, attached to this example as a
    % supporting file, to extract a downsampled point cloud and its labels
    % from the dense point cloud.
    [ptCloudOut,labelsOut] = helperDownsamplePoints(pc, ...
    classification,numPoints);

    % Make the spatial extent of the dense point cloud and the sparse point
    % cloud same.
    limits = [ptCloudOut.XLimits;ptCloudOut.YLimits;...
                    ptCloudOut.ZLimits];
    ptCloudSparseLocation = ptCloudOut.Location;
    ptCloudSparseLocation(1:2,:) = limits(:,1:2)';
    ptCloudSparseUpdated = pointCloud(ptCloudSparseLocation, ...
        'Intensity',ptCloudOut.Intensity, ...
        'Color',ptCloudOut.Color, ...
        'Normal',ptCloudOut.Normal);

    % Use the helperNormalizePointCloud function, attached to this example as
    % a supporting file, to normalize the point cloud between 0 and 1.    
    ptCloudOutSparse = helperNormalizePointCloud( ...
        ptCloudSparseUpdated);
    cellout{i,1} = ptCloudOutSparse.Location;

    % Permuted output.
    cellout{i,2} = permute(categorical(labelsOut,labelIDs,classNames),[1 3 2]);

    % General output.
    dataout{i,1} = ptCloudOutSparse;
    dataout{i,2} = labelsOut;
end
end

%%
function data = helperTransformToTestData(data)
if ~iscell(data)
    data = {data};
end
numObservations = size(data,1);
for i = 1:numObservations
    tmp = data{i,1}.Location;
    data{i,1} = tmp;
end
end