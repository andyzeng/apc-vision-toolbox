% Test on MIT-Princeton APC 2016 Benchmark
%
% Start ROS services marvin_convnet and pose_estimation before running this
% script (and realsense_camera if necessary)
% Set marvin_convnet to mode 2 to read RGB-D data from disk
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
rgbdUtilsPath = '/home/andyz/apc/toolbox/rgbd-utils'; % Directory of RGB-D toolbox utilities
resultVisPath = 'results'; % Directory to save visualizations of pose estimation results

% Load benchmark scenes and labels
sceneList = {};
fid = fopen(fullfile(benchmarkPath,'info.txt'));
while ~feof(fid)
    scenePath = fullfile(benchmarkPath,fscanf(fid,'%s,'));
    if ~feof(fid)
        sceneList{length(sceneList)+1} = scenePath(1:(end-1));
    end
end
fclose(fid);

% Add paths and create directories
addpath(genpath(rgbdUtilsPath));
if ~exist(tmpDataPath,'file')
    mkdir(tmpDataPath);
end
if ~exist(resultVisPath,'file')
    mkdir(resultVisPath);
end

% Loop through benchmark scenes
for sceneIdx = 1:length(sceneList)
  scenePath = sceneList{sceneIdx};
  fprintf(sprintf('[Test] %s\n',scenePath));
  
  % Remove all files in temporary folder used by marvin_convnet
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
  
  % Call apc_posest to do object pose estimation for the sequence
  [client,reqMsg] = rossvcclient('/pose_estimation');
  reqMsg.SceneFiles = tmpDataPath;
  if ~isempty(strfind(scenePath,'office'))
    reqMsg.CalibrationFiles = '/home/andyz/apc/toolbox/data/benchmark/office/calibration';
  elseif ~isempty(strfind(scenePath,'warehouse'))
    reqMsg.CalibrationFiles = '/home/andyz/apc/toolbox/data/benchmark/warehouse/calibration';
  end
  reqMsg.DoCalibration = true;
  response = call(client,reqMsg);
  
%   % Copy result and visualization files to results folder
%   resultFiles = dir(fullfile(tmpDataPath,'*.result.txt'));
%   resultImages = dir(fullfile(tmpDataPath,'*.result.png'));
%   visFiles = dir('*.ply');
%   visObjNames = {};
%   visObjScores = {};
%   for resultIdx = 1:length(resultFiles)
%     currResultFile = resultFiles(resultIdx).name;
%     currResultFilenameDotIdx = strfind(currResultFile,'.');
%     currObjName = currResultFile(1:(currResultFilenameDotIdx(1)-1));
%     currResultFileMatrix = dlmread(fullfile(tmpDataPath,currResultFile));
%     currObjScore = currResultFileMatrix(length(currResultFileMatrix));
%     visObjNames{length(visObjNames)+1} = currObjName;
%     visObjScores{length(visObjScores)+1} = currObjScore;
% %     copyfile(fullfile(tmpPath,resultFiles(resultIdx).name),fullfile(resultPath,[testDir(seqIdx).name,'.',resultFiles(resultIdx).name]));
%   end
%   for resultIdx = 1:length(resultImages)
%     currResultFile = resultImages(resultIdx).name;
%     visResultImage = imread(fullfile(tmpDataPath,currResultFile));
%     [sortVals,sortIdx] = sortrows(cell2mat(visObjScores)',-1);
%     visResultImage = insertText(visResultImage,[10 10],'Confidence :  Object ID','TextColor','white','BoxColor','black');
%     textY = 32;
%     for objIdx = 1:length(visObjNames)
%       if ~isempty(strfind(currResultFile,visObjNames{sortIdx(objIdx)}))%resultIdx == sortIdx(objIdx)
%         visResultImage = insertText(visResultImage,[10 textY],sprintf('%f  :  %s',visObjScores{sortIdx(objIdx)},visObjNames{sortIdx(objIdx)}),'TextColor','black','BoxColor','green');
%       else
%         visResultImage = insertText(visResultImage,[10 textY],sprintf('%f  :  %s',visObjScores{sortIdx(objIdx)},visObjNames{sortIdx(objIdx)}),'TextColor','white','BoxColor','black');
%       end
%       textY = textY + 22;
%     end
%     imwrite(visResultImage,fullfile(resultPath,strcat(testDir(sceneIdx).name,'.',resultImages(resultIdx).name)));
% %     copyfile(fullfile(tmpPath,resultImages(resultIdx).name),fullfile(resultPath,[testDir(seqIdx).name,'.',resultImages(resultIdx).name]));
%   end
% %   copyfile(fullfile(tmpPath,'frame-000007.color.png'),fullfile(resultPath,[testDir(seqIdx).name,'.','frame-000007.color.png']));
%   for visFileIdx = 1:length(visFiles)
%     movefile(visFiles(visFileIdx).name,fullfile(resultPath,[testDir(sceneIdx).name,'.',visFiles(visFileIdx).name]));
%   end
  
end
















