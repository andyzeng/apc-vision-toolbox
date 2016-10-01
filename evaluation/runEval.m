% Test full vision system on our benchmark dataset 'Shelf & Tote'
%
% Start ROS services /marvin_convnet detect and /pose_estimation before running this
% script (and realsense_camera if necessary)
% Run in same Matlab instance as where ROS service /pose_estimation was started
%
% ---------------------------------------------------------
% Copyright (c) 2016, Andy Zeng
% 
% This file is part of the APC Vision Toolbox and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------

% User configurations (change me)
toolboxPath = '/home/andyz/apc/toolbox'; % Directory of toolbox utilities
tmpDataPath = '/home/andyz/apc/toolbox/data/tmp'; % Temporary directory used by marvin_convnet, where all RGB-D images and detection masks are saved
benchmarkPath = '/home/andyz/apc/toolbox/data/benchmark'; % Benchmark dataset directory
modelsPath = '/home/andyz/apc/toolbox/ros-packages/catkin_ws/src/pose_estimation/src/models/objects'; % Directory holding pre-scanned object models
savetoPath = './predictions.mat'; % Where to save predictions

% Add paths and create directories
cd(fileparts(which('runEval.m')));
addpath('.');
addpath(genpath(fullfile(toolboxPath,'rgbd-utils')));
addpath(genpath(fullfile(toolboxPath,'vis-utils')));
if ~exist(tmpDataPath,'file')
    mkdir(tmpDataPath);
end
if ~exist('results','file')
    mkdir('results');
end

% Get benchmark scene list
load(fullfile(benchmarkPath,'scenes.mat'));

% Get object names
load('objectNames.mat');
  
% Get set of colors for visualizations (from vis-utils)
load('colorPalette.mat');

% Loop through benchmark scenes
predictions = {};
for sceneIdx = 1:length(scenes)
    scenePath = fullfile(benchmarkPath,scenes{sceneIdx});
    fprintf(sprintf('[Test] %s\n',scenePath));

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

    % Load scene
    sceneData = loadScene(scenePath);
    numFrames = length(sceneData.colorFrames);
    
    % Calibrate scene
    slashStrInd = strfind(scenePath,'/');
    sceneData = loadCalib(fullfile(scenePath(1:slashStrInd(end-2)),'calibration'),sceneData);
    
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
        
        % For each RGB-D frame, call marvin_convnet to do 2D object segmentation
        try
            [client,reqMsg] = rossvcclient('/marvin_convnet');
        catch
            cd('/home/andyz/apc/toolbox/ros-packages/catkin_ws/src/pose_estimation/src');
            fprintf('[Error] Restarting services\n')
            startService
            cd('/home/andyz/apc/toolbox/evaluation');
            [client,reqMsg] = rossvcclient('/marvin_convnet');
        end
        reqMsg.BinId = binNum;
        reqMsg.ObjectNames = sceneData.objects;
        reqMsg.FrameId = frameIdx;
        response = call(client,reqMsg);
        
%         % Use perfect segmentation 
%         if ~exist(fullfile(tmpDataPath,'masks'),'file')
%             mkdir(fullfile(tmpDataPath,'masks'),'s');
%         end
%         uniqueObjList = unique(sceneData.objects);
%         currGroundTruthSegm = imread(fullfile(tmpDataPath,'segm',sprintf('frame-%06d.segm.png',frameIdx)));
%         for objIdx = 1:length(uniqueObjList)
%             objId = find(~cellfun(@isempty,strfind(objNames,uniqueObjList{objIdx})));
%             currGroundTruthMask = double(currGroundTruthSegm)./6 == objId;
%             imwrite(uint16(double(currGroundTruthMask).*65535),fullfile(tmpDataPath,'masks',sprintf('frame-%06d.%s.mask.png',frameIdx,uniqueObjList{objIdx})));
%         end
    end
    fprintf('\n');

    % Call pose_estimation to do 6D object pose estimation for the sequence
    try
        [client,reqMsg] = rossvcclient('/pose_estimation');
    catch
        fprintf('[Error] Restarting services\n')
        cd('/home/andyz/apc/toolbox/ros-packages/catkin_ws/src/pose_estimation/src');
        startService
        cd('/home/andyz/apc/toolbox/evaluation');
        [client,reqMsg] = rossvcclient('/pose_estimation');
    end
    reqMsg.SceneFiles = tmpDataPath;
    if ~isempty(strfind(scenePath,'office'))
        reqMsg.CalibrationFiles = fullfile(benchmarkPath,'office','calibration');
    elseif ~isempty(strfind(scenePath,'warehouse'))
        reqMsg.CalibrationFiles = fullfile(benchmarkPath,'warehouse','calibration');
    end
    reqMsg.DoCalibration = true;
    try
        respMsg= call(client,reqMsg);
    catch
    end
    
    % Evaluate 2D Segmentation (compute precision recall)
    uniqueObjList = unique(sceneData.objects);
    segmPr = zeros(numFrames,length(uniqueObjList));
    segmRc = zeros(numFrames,length(uniqueObjList));
    for frameIdx = 1:numFrames
        currGroundTruthSegm = imread(fullfile(tmpDataPath,'segm',sprintf('frame-%06d.segm.png',frameIdx-1)));
        for objIdx = 1:length(uniqueObjList)
            currPredSegm = imread(fullfile(tmpDataPath,'masks',sprintf('frame-%06d.%s.mask.png',frameIdx-1,uniqueObjList{objIdx})));
            currSegmThresh = dlmread(fullfile(tmpDataPath,'masks',sprintf('%s.thresh.txt',uniqueObjList{objIdx})));
            objId = find(~cellfun(@isempty,strfind(objectNames,uniqueObjList{objIdx})));
            currGroundTruthMask = double(currGroundTruthSegm)./6 == objId;
            currPredMask = double(currPredSegm)./65535 > currSegmThresh;
            tp = sum(currPredMask(:) & currGroundTruthMask(:));
            fp = sum(currPredMask(:) & ~currGroundTruthMask(:));
            fn = sum(~currPredMask(:) & currGroundTruthMask(:));
            segmPr(frameIdx,objIdx) = (tp/(tp+fp));
            segmRc(frameIdx,objIdx) = (tp/(tp+fn));
            if tp < 200
                segmPr(frameIdx,objIdx) = NaN;
            end
        end
    end
    segmRc(find(isnan(segmPr))) = NaN;
    segmPr = nanmean(segmPr);
    segmRc = nanmean(segmRc);

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
    [~,sortIdx] = sortrows(confScores,-1);

    % Get random colors per object in the scene
    randColorIdx = randperm(length(colorPalette),length(results));
    
    % Create canvases for visualization
    if strcmp(sceneData.env,'shelf')
        canvas = sceneData.colorFrames{8};
        canvasSeg = uint8(ones(size(canvas))*255);
        canvasPose = canvas;
        canvasPose = insertText(canvasPose,[10 10],'Confidence  :  Object','Font','LucidaSansDemiBold','FontSize',12,'TextColor','white','BoxColor','black');
    else
        canvas1 = sceneData.colorFrames{5};
        canvas2 = sceneData.colorFrames{14};
        canvasSeg1 = uint8(ones(size(canvas1))*255);
        canvasPose1 = canvas1;
        canvasSeg2 = uint8(ones(size(canvas2))*255);
        canvasPose2 = canvas2;
        canvasPose1 = insertText(canvasPose1,[10 10],'Confidence  :  Object','Font','LucidaSansDemiBold','FontSize',12,'TextColor','white','BoxColor','black');
    end
    
    % Loop through each object to save and visualize predicted object pose
    textPosY = 32;
    for resultIdx = 1:length(resultFiles)
        currResult = results{sortIdx(resultIdx)};
        objColor = colorPalette{randColorIdx(resultIdx)};
        
        % Save predicted pose estimation results to file
        slashIdx = strfind(scenePath,'/');
        tmpPrediction.sceneName = scenePath((slashIdx(end-3)+1):end);
        tmpPrediction.objectName = currResult.objName;
        tmpPrediction.objectPose = currResult.objPoseWorld;
        tmpPrediction.confidence = currResult.confScore;
        tmpPrediction.segmPrecision = segmPr(find(~cellfun(@isempty,strfind(uniqueObjList,currResult.objName))));
        tmpPrediction.segmRecall = segmRc(find(~cellfun(@isempty,strfind(uniqueObjList,currResult.objName))));
        predictions{length(predictions)+1} = tmpPrediction;
        
        % Load pre-scanned object point cloud
        objPointCloud = pcread(fullfile(modelsPath,sprintf('%s.ply',currResult.objName)));

        % Draw projected object model and bounding box
        if currResult.confScore > 0
            if strcmp(sceneData.env,'shelf')
                [canvasSeg,canvasPose] = showObjectPose(currResult.objName,...
                                                        canvasSeg, ...
                                                        canvasPose, ...
                                                        sceneData.colorFrames{8}, ...
                                                        sceneData.depthFrames{8}, ...
                                                        sceneData.extCam2World{8}, ...
                                                        sceneData.colorK, ...
                                                        objPointCloud, ...
                                                        currResult.objPoseWorld, ...
                                                        objColor);                             
                canvasPose = insertText(canvasPose,[10 textPosY],sprintf('  %f  :  %s',currResult.confScore,currResult.objName),'Font','LucidaSansDemiBold','FontSize',12,'TextColor',objColor,'BoxColor','black');
            else
                [canvasSeg1,canvasPose1] = showObjectPose(currResult.objName,...
                                                          canvasSeg1, ...
                                                          canvasPose1, ...
                                                          sceneData.colorFrames{5}, ...
                                                          sceneData.depthFrames{5}, ...
                                                          sceneData.extCam2World{5}, ...
                                                          sceneData.colorK, ...
                                                          objPointCloud, ...
                                                          currResult.objPoseWorld, ...
                                                          objColor);
                [canvasSeg2,canvasPose2] = showObjectPose(currResult.objName,...
                                                          canvasSeg2, ...
                                                          canvasPose2, ...
                                                          sceneData.colorFrames{14}, ...
                                                          sceneData.depthFrames{14}, ...
                                                          sceneData.extCam2World{14}, ...
                                                          sceneData.colorK, ...
                                                          objPointCloud, ...
                                                          currResult.objPoseWorld, ...
                                                          objColor);                  
                canvasPose1 = insertText(canvasPose1,[10 textPosY],sprintf('  %f  :  %s',currResult.confScore,currResult.objName),'Font','LucidaSansDemiBold','FontSize',12,'TextColor',objColor,'BoxColor','black');
                canvasSeg = [canvasSeg1;canvasSeg2];
                canvasPose = [canvasPose1;canvasPose2];
            end
        end
        
        textPosY = textPosY + 22;
    end
    
    % Save visualization
    visFileName = scenes{sceneIdx};
    visFileName = visFileName(1:end);
    visFileName = strrep(visFileName,'/','_');
    imwrite(canvasPose,fullfile('results',sprintf('%s.png',visFileName)));
end

% Save prediction file
predictions = predictions';
save(savetoPath,'predictions');














