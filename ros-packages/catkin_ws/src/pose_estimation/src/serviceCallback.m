function resp = serviceCallback(~, reqMsg, respMsg)
% Matlab service callback function for pose_estimation
% Reads scene RGB-D data with object segmentation results, and predicts 
% 6D object poses
%
% ---------------------------------------------------------
% Copyright (c) 2016, Andy Zeng
% 
% This file is part of the APC Vision Toolbox and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------

global visPath;
global savePointCloudVis; 
global saveResultImageVis;
global objNames
global objModels
global emptyShelfModels;
global emptyToteModel;
global useGPU;

% Load RGB-D frames for scene
scenePath = reqMsg.SceneFiles;
sceneData = loadScene(scenePath);
if reqMsg.DoCalibration
    sceneData = loadCalib(reqMsg.CalibrationFiles,sceneData);
end

% Fill holes in depth frames for scene
for frameIdx = 1:length(sceneData.depthFrames)
    sceneData.depthFrames{frameIdx} = fillHoles(sceneData.depthFrames{frameIdx});
end

% Load scene point cloud
fprintf('    [Processing] Loading scene point clouds\n');
scenePointCloud = getScenePointCloud(sceneData);

% Load and align empty bin/tote point cloud to observation
binIds = 'ABCDEFGHIJKL';
if strcmp(sceneData.env,'shelf')
    backgroundPointCloud = emptyShelfModels{strfind(binIds,sceneData.binId)};
else
    backgroundPointCloud = emptyToteModel;
end
if useGPU
    [tform,backgroundPointCloud] = pcregrigidGPU(backgroundPointCloud,scenePointCloud,'InlierRatio',0.8,'MaxIterations',200,'Tolerance',[0.0001,0.0009],'Verbose',false,'Extrapolate',true);
else
    [tform,backgroundPointCloud] = pcregrigid(backgroundPointCloud,scenePointCloud,'InlierRatio',0.8,'MaxIterations',200,'Tolerance',[0.0001,0.0009],'Verbose',false,'Extrapolate',true);
end
extBin2Bg = inv(tform.T');

if savePointCloudVis
  pcwrite(scenePointCloud,fullfile(visPath,'vis.pc.scene'),'PLYFormat','binary');
  pcwrite(backgroundPointCloud,fullfile(visPath,'vis.pc.bg'),'PLYFormat','binary');
end

% Parse object names
objList = sceneData.objects;
uniqueObjectList = unique(objList);
for objIdx = 1:length(uniqueObjectList)
%     tic;
    objName = uniqueObjectList{objIdx};
    objNum = sum(ismember(objList,objName));
    objModel = objModels{find(ismember(objNames,objName))};
    if isempty(objModel)
        fprintf('Error: object model not loaded for %s!\n',objName);
    end

    % Do 6D pose estimation for each object and save the results
    objPoses = getObjectPose(scenePath,sceneData,scenePointCloud,backgroundPointCloud,extBin2Bg,objName,objModel,objNum);
    for duplicateIdx = 1:length(objPoses)
        respMsg.Objects = [respMsg.Objects; objPoses(duplicateIdx)];
    end
%     toc;
end
resp = true;
end
