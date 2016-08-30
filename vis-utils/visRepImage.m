clear all; close all;

% Config options 
labelListFile = 'labelList.csv';
dataPath = '/home/andyz/apc/toolbox/data/benchmark';
objCloudPath = '/home/andyz/apc/toolbox/data/pointclouds/objects';
toolboxPath = '/home/andyz/apc/toolbox';

% Add toolbox paths
addpath(genpath(fullfile(toolboxPath,'rgbd-utils')));

% Read label data from annotations
labelList = parseLabels(labelListFile);

% Get set of colors
colorPalette = getColorPalette();

for entryIdx = 1:length(labelList)
    labelEntry = labelList{entryIdx};
    if isempty(labelEntry)
        continue;
    end
    
    % Find all object labels in the current RGB-D sequence
    objList = {};
    objPose = {};
    objList{1} = labelEntry.objName;
    objPose{1} = labelEntry.objPose;
    
    % Find RGB-D sequence path 
    switch labelEntry.seqName(1:3)
        case 'ots'
            scenePath = fullfile(dataPath,'office','test','shelf');
            calibPath = fullfile(dataPath,'office','calibration');
        case 'ott'
            scenePath = fullfile(dataPath,'office','test','tote');
            calibPath = fullfile(dataPath,'office','calibration');
        case 'wcp'
            scenePath = fullfile(dataPath,'warehouse','competition','shelf');
            calibPath = fullfile(dataPath,'warehouse','calibration');
        case 'wcs'
            scenePath = fullfile(dataPath,'warehouse','competition','tote');
            calibPath = fullfile(dataPath,'warehouse','calibration');
        case 'wps'
            scenePath = fullfile(dataPath,'warehouse','practice','shelf');
            calibPath = fullfile(dataPath,'warehouse','calibration');
        case 'wpt'
            scenePath = fullfile(dataPath,'warehouse','practice','tote');
            calibPath = fullfile(dataPath,'warehouse','calibration');
    end
    scenePath = fullfile(scenePath,sprintf('scene-%s',labelEntry.seqName(4:7)));
    
    % Load RGB-D sequence data
    sceneData = loadScene(scenePath);
    
    % Apply calibrated camera poses to sequence data
    sceneData = loadCalib(calibPath,sceneData);
    
    % Create point cloud from sequence data
    scenePointCloud = getScenePointCloud(sceneData);
    
    % Compute coordinate frame transform from world space to web space
    extWorld2Web = eye(4);
    extWorld2Web(1,4) = -mean(scenePointCloud.XLimits);
    extWorld2Web(2,4) = -mean(scenePointCloud.YLimits);
    extWorld2Web(3,4) = -min(scenePointCloud.ZLimits);
    axesSwap = eye(4);
    axesSwap(1:3,1:3) = vrrotvec2mat([1,0,0,-pi/2]);
    extWorld2Web = axesSwap*extWorld2Web;
    
    % Get center camera position
    if strcmp(sceneData.env,'shelf')
        camLoc = sceneData.extCam2World{8};
        camLoc = camLoc(1:3,4);
    else
        camLoc = (sceneData.extCam2World{8}+sceneData.extCam2World{11})./2;
        camLoc = camLoc(1:3,4);
    end
        
    % Sort objects in the scene from back to front (X direction for shelf,
    % Z direction for tote)
    objOrderVal = zeros(1,length(objList));
    for objIdx=1:length(objList)
        objPoseWeb = reshape(objPose{objIdx},[4,4]);
        objPoseWorld = inv(extWorld2Web)*objPoseWeb;
        objOrderVal(objIdx) = sqrt(sum((objPoseWorld(1:3,4)-camLoc).^ 2));
    end
    if strcmp(sceneData.env,'shelf')
        [~,objOrder] = sort(objOrderVal,'descend');
    else
        [~,objOrder] = sort(objOrderVal,'ascend');
    end
        
    % Get random colors
    randColorIdx = randperm(length(colorPalette),length(objList));
    
    % Visualize all objects in the scene
    if strcmp(sceneData.env,'shelf')
        canvas = sceneData.colorFrames{10};
        canvasSeg = uint8(ones(size(canvas))*255);
        canvasPose = canvas;
    else
        canvas1 = sceneData.colorFrames{5};
        canvas2 = sceneData.colorFrames{14};
        canvasSeg1 = uint8(ones(size(canvas1))*255);
        canvasPose1 = canvas1;
        canvasSeg2 = uint8(ones(size(canvas2))*255);
        canvasPose2 = canvas2;
    end
    for objIdx=objOrder
    
        % Load object point cloud
        objCloud = pcread(fullfile(objCloudPath,sprintf('%s.ply',objList{objIdx})));

        % Compute object pose in world space
        objPoseWeb = reshape(objPose{objIdx},[4,4]);
        objPoseWorld = inv(extWorld2Web)*objPoseWeb;

        % Visualize object
        objColor = colorPalette{randColorIdx(objIdx)};
        if strcmp(sceneData.env,'shelf')
            [canvasSeg,canvasPose] = dispObjPose(canvasSeg, ...
                                                 canvasPose, ...
                                                 sceneData.colorFrames{10}, ...
                                                 sceneData.depthFrames{10}, ...
                                                 sceneData.extCam2World{10}, ...
                                                 sceneData.colorK, ...
                                                 objCloud, ...
                                                 objPoseWorld, ...
                                                 [42 127 255]);
        else
            [canvasSeg1,canvasPose1] = dispObjPose(canvasSeg1, ...
                                                   canvasPose1, ...
                                                   sceneData.colorFrames{5}, ...
                                                   sceneData.depthFrames{5}, ...
                                                   sceneData.extCam2World{5}, ...
                                                   sceneData.colorK, ...
                                                   objCloud, ...
                                                   objPoseWorld, ...
                                                   objColor);
            [canvasSeg2,canvasPose2] = dispObjPose(canvasSeg2, ...
                                                   canvasPose2, ...
                                                   sceneData.colorFrames{14}, ...
                                                   sceneData.depthFrames{14}, ...
                                                   sceneData.extCam2World{14}, ...
                                                   sceneData.colorK, ...
                                                   objCloud, ...
                                                   objPoseWorld, ...
                                                   objColor);
            canvasSeg = [canvasSeg1;canvasSeg2];
            canvasPose = [canvasPose1;canvasPose2];
        end
    end
    imwrite(canvasPose,fullfile(sprintf('%s.pose.png',labelEntry.seqName)));
end