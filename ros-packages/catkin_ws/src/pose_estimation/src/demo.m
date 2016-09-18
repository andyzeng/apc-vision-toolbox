% User configurations (change me)
toolboxPath = '/home/andyz/apc/toolbox'; % Directory of toolbox utilities
tmpDataPath = '/home/andyz/apc/toolbox/data/tmp'; % Temporary directory used by marvin_convnet, where all RGB-D images and detection masks are saved
modelsPath = './models/objects'; % Directory holding pre-scanned object models
scenePath = fullfile(toolboxPath,'data/sample/scene-0000'); % Directory holding the RGB-D data of the sample scene
calibPath = fullfile(toolboxPath,'data/sample/calibration'); % Directory holding camera pose calibration data for the sample scene

% Add paths and create directories
addpath(genpath(fullfile(toolboxPath,'rgbd-utils')));
addpath(genpath(fullfile(toolboxPath,'vis-utils')));

% Get set of colors for visualizations (from vis-utils)
load('colorPalette.mat');

% Remove all files in temporary folder used by marvin_convnet
if exist(fullfile(tmpDataPath,'raw'),'file')
    rmdir(fullfile(tmpDataPath,'raw'),'s');
end
if exist(fullfile(tmpDataPath,'results'),'file')
    rmdir(fullfile(tmpDataPath,'results'),'s');
end
if exist(fullfile(tmpDataPath,'HHA'),'file')
    rmdir(fullfile(tmpDataPath,'HHA'),'s');
end
if exist(fullfile(tmpDataPath,'masks'),'file')
    rmdir(fullfile(tmpDataPath,'masks'),'s');
end
if exist(fullfile(tmpDataPath,'segm'),'file')
    rmdir(fullfile(tmpDataPath,'segm'),'s');
end
delete(fullfile(tmpDataPath,'*'));

% Copy current scene to temporary folder used by marvin_convnet
copyfile(fullfile(scenePath,'*'),tmpDataPath);

% Load sample scene data
fprintf('    [Processing] Loading scene RGB-D data\n');
sceneData = loadScene(tmpDataPath);
numFrames = length(sceneData.colorFrames);

% Calibrate scene
sceneData = loadCalib(calibPath,sceneData);

% Fill holes in depth frames for scene and make HHA
if ~exist(fullfile(tmpDataPath,'HHA'),'file')
    fprintf('    [Processing] Creating HHA maps from depth data\n');
    mkdir(fullfile(tmpDataPath,'HHA'));
    for frameIdx = 1:length(sceneData.depthFrames)
        sceneData.depthFrames{frameIdx} = fillHoles(sceneData.depthFrames{frameIdx});
        HHA = getHHA(sceneData.env,sceneData.depthFrames{frameIdx},sceneData.colorK,sceneData.extCam2World{frameIdx},sceneData.extBin2World);
        imwrite(HHA,fullfile(tmpDataPath,'HHA',sprintf('frame-%06d.HHA.png',frameIdx-1)));
    end
end

% Call marvin_convnet to do 2D object segmentation for each RGB-D frame
fprintf('    [Segmentation] Frame ');
if strcmp(sceneData.env,'shelf')
    binIds = 'ABCDEFGHIJKL';
    binNum = strfind(binIds,sceneData.binId)-1;
else
    binNum = -1;
end
for frameIdx = 0:(numFrames-1)
    fprintf('%d ',frameIdx);
    [client,reqMsg] = rossvcclient('/marvin_convnet');
    reqMsg.BinId = binNum;
    reqMsg.ObjectNames = sceneData.objects;
    reqMsg.FrameId = frameIdx;
    response = call(client,reqMsg);
end
fprintf('\n');

% Call pose_estimation to do 6D object pose estimation for the sequence
[client,reqMsg] = rossvcclient('/pose_estimation');
reqMsg.SceneFiles = tmpDataPath;
reqMsg.CalibrationFiles = calibPath;
reqMsg.DoCalibration = true;
try
    respMsg= call(client,reqMsg);
catch
end

fprintf('    [Visualization] Drawing predicted object poses\n');
% Load results (predicted 6D object poses)
resultFiles = dir(fullfile(tmpDataPath,'results/*.result.txt'));
results = cell(1,length(resultFiles));
confScores = [];
for resultIdx = 1:length(resultFiles)
    tmpResultFile = resultFiles(resultIdx).name;
    tmpResultFilenameDotIdx = strfind(tmpResultFile,'.');
    tmpResult.objName = tmpResultFile(1:(tmpResultFilenameDotIdx(1)-1));
    tmpResult.objNum = str2double(tmpResultFile((tmpResultFilenameDotIdx(1)+1):(tmpResultFilenameDotIdx(2)-1)));
    tmpResult.objPoseWorld = eye(4);
    tmpResult.objPoseWorld(1:3,4) = dlmread(fullfile(tmpDataPath,'results',tmpResultFile),'\t',[1,0,1,2])';
    objPoseRotQuat = dlmread(fullfile(tmpDataPath,'results',tmpResultFile),'\t',[4,0,4,3]);
    tmpResult.objPoseWorld(1:3,1:3) = quat2rotm([objPoseRotQuat(4),objPoseRotQuat(1:3)]);
    tmpResult.confScore = dlmread(fullfile(tmpDataPath,'results',tmpResultFile),'\t',[28,0,28,0]);
    confScores = [confScores;tmpResult.confScore];
    results{resultIdx} = tmpResult;
end

% Sort results by confidence scores
[~,sortIdx] = sortrows(confScores,1);

% Get random colors per object in the scene
randColorIdx = randperm(length(colorPalette),length(results));

% Create canvas for visualization
canvas = sceneData.colorFrames{8};
canvasSeg = uint8(ones(size(canvas))*255);
canvasPose = canvas;
canvasPose = insertText(canvasPose,[10 10],'Confidence  :  Object','Font','LucidaSansDemiBold','FontSize',12,'TextColor','white','BoxColor','black');

% Loop through each object to save and visualize predicted object pose
uniqueObjList = unique(sceneData.objects);
textPosY = 32;
for resultIdx = 1:length(resultFiles)
    currResult = results{sortIdx(resultIdx)};
    objColor = colorPalette{randColorIdx(resultIdx)};

    % Load pre-scanned object point cloud
    objPointCloud = pcread(fullfile(modelsPath,sprintf('%s.ply',currResult.objName)));

    % Draw projected object model and bounding box
    [canvasSeg,canvasPose] = showObjectPose(currResult.objName, canvasSeg, canvasPose, sceneData.colorFrames{8}, sceneData.depthFrames{8}, sceneData.extCam2World{8}, sceneData.colorK, objPointCloud, currResult.objPoseWorld, objColor);                             
    canvasPose = insertText(canvasPose,[10 textPosY],sprintf('  %f  :  %s',currResult.confScore,currResult.objName),'Font','LucidaSansDemiBold','FontSize',12,'TextColor',objColor,'BoxColor','black');
    textPosY = textPosY + 22;
end

imshow(canvasPose); title('Predicted 6D Object Poses');

