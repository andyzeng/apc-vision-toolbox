% Test on MIT-Princeton APC 2016 Benchmark
%
% Start ROS services marvin_convnet and pose_estimation before running this
% script (and realsense_camera if necessary)
% Set marvin_convnet to _service_mode:=2 to read RGB-D data from disk
% Run in same Matlab instance as where ROS service pose_estimation was started
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
tmpDataPath = '/home/andyz/apc/toolbox/data/tmp'; % Temporary directory used by marvin_convnet, where all RGB-D images and detection masks are saved
benchmarkPath = '/home/andyz/apc/toolbox/data/benchmark'; % Benchmark directory
modelsPath = '/home/andyz/apc/toolbox/ros-packages/catkin_ws/src/pose_estimation/src/models/objects'; % Directory holding pre-scanned object models
rgbdUtilsPath = '/home/andyz/apc/toolbox/rgbd-utils'; % Directory of RGB-D toolbox utilities
visUtilsPath = '/home/andyz/apc/toolbox/vis-utils'; % Directory of visualization utilities
resultVisPath = 'results'; % Directory to save visualizations of pose estimation results

% Load benchmark scenes and labels
sceneList = {};
sceneNames = {};
fid = fopen(fullfile(benchmarkPath,'info.txt'));
while ~feof(fid)
    sceneName = fscanf(fid,'%s,');
    scenePath = fullfile(benchmarkPath,sceneName);
    if ~feof(fid)
        sceneList{length(sceneList)+1} = scenePath(1:(end-1));
        sceneNames{length(sceneNames)+1} = sceneName;
    end
end
fclose(fid);

% Add paths and create directories
addpath(genpath(rgbdUtilsPath));
addpath(genpath(visUtilsPath));
if ~exist(tmpDataPath,'file')
    mkdir(tmpDataPath);
end
if ~exist(resultVisPath,'file')
    mkdir(resultVisPath);
end
  
% Get set of colors for visualizations
colorPalette = getColorPalette();

% Loop through benchmark scenes
for sceneIdx = 1:length(sceneList)
    scenePath = sceneList{sceneIdx};
    fprintf(sprintf('[Test] %s\n',scenePath));

    % Remove all files in temporary folder used by marvin_convnet
    if exist(fullfile(tmpDataPath,'raw'),'file')
        rmdir(fullfile(tmpDataPath,'raw'),'s');
    end
    if exist(fullfile(tmpDataPath,'results'),'file')
        rmdir(fullfile(tmpDataPath,'results'),'s');
    end
    delete(fullfile(tmpDataPath,'*'));

    % Copy current scene to temporary folder used by marvin_convnet
    copyfile(fullfile(scenePath,'*'),tmpDataPath);

    % Load scene
    sceneData = loadScene(scenePath);
    numFrames = length(sceneData.colorFrames);

    % Call marvin_convnet to do 2D object segmentation for each RGB-D frame
    fprintf('    [Segmentation] Frame ');
    binIds = 'ABCDEFGHIJKL';
    binNum = strfind(binIds,sceneData.binId)-1;
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
    if ~isempty(strfind(scenePath,'office'))
        reqMsg.CalibrationFiles = '/home/andyz/apc/toolbox/data/benchmark/office/calibration';
    elseif ~isempty(strfind(scenePath,'warehouse'))
        reqMsg.CalibrationFiles = '/home/andyz/apc/toolbox/data/benchmark/warehouse/calibration';
    end
    reqMsg.DoCalibration = true;
    respMsg= call(client,reqMsg);

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
    else
        canvas1 = sceneData.colorFrames{5};
        canvas2 = sceneData.colorFrames{14};
        canvasSeg1 = uint8(ones(size(canvas1))*255);
        canvasPose1 = canvas1;
        canvasSeg2 = uint8(ones(size(canvas2))*255);
        canvasPose2 = canvas2;
    end
    
    % Loop through each object and visualize predicted object pose
    canvasPose = insertText(canvasPose,[10 10],'Confidence  :  Object','Font','LucidaSansDemiBold','FontSize',16,'TextColor','white','BoxColor','black');
    textPosY = 38;
    for resultIdx = 1:length(resultFiles)
        currResult = results{sortIdx(resultIdx)};
        objColor = colorPalette{randColorIdx(resultIdx)};
        
        % Load pre-scanned object point cloud
        objPointCloud = pcread(fullfile(modelsPath,sprintf('%s.ply',currResult.objName)));


        % Draw projected object model and bounding box
        if strcmp(sceneData.env,'shelf')
            [canvasSeg,canvasPose] = dispObjPose(canvasSeg, ...
                                                 canvasPose, ...
                                                 sceneData.colorFrames{8}, ...
                                                 sceneData.depthFrames{8}, ...
                                                 sceneData.extCam2World{8}, ...
                                                 sceneData.colorK, ...
                                                 objPointCloud, ...
                                                 currResult.objPoseWorld, ...
                                                 objColor);
        else
            [canvasSeg1,canvasPose1] = dispObjPose(canvasSeg1, ...
                                                   canvasPose1, ...
                                                   sceneData.colorFrames{5}, ...
                                                   sceneData.depthFrames{5}, ...
                                                   sceneData.extCam2World{5}, ...
                                                   sceneData.colorK, ...
                                                   objPointCloud, ...
                                                   currResult.objPoseWorld, ...
                                                   objColor);
            [canvasSeg2,canvasPose2] = dispObjPose(canvasSeg2, ...
                                                   canvasPose2, ...
                                                   sceneData.colorFrames{14}, ...
                                                   sceneData.depthFrames{14}, ...
                                                   sceneData.extCam2World{14}, ...
                                                   sceneData.colorK, ...
                                                   objPointCloud, ...
                                                   currResult.objPoseWorld, ...
                                                   objColor);
            canvasSeg = [canvasSeg1;canvasSeg2];
            canvasPose = [canvasPose1;canvasPose2];
        end
        
        canvasPose = insertText(canvasPose,[10 textPosY],sprintf('  %f  :  %s',currResult.confScore,currResult.objName),'Font','LucidaSansDemiBold','FontSize',16,'TextColor',objColor,'BoxColor','black');
        textPosY = textPosY + 28;
    end
    
    % Save visualization
    visFileName = sceneNames{sceneIdx};
    visFileName = visFileName(1:(end-1));
    visFileName = strrep(visFileName,'/','_');
    imwrite(canvasPose,fullfile(resultVisPath,sprintf('%s.png',visFileName)));
end
















