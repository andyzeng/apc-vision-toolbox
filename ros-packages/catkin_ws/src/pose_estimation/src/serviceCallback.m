function resp = serviceCallback(~, reqMsg, respMsg)

global visPath;
global savePointCloudVis; 
global saveResultImageVis;
global objNames
global objModels
global emptyShelfModels;
global emptyToteModel;

% Load RGB-D frames for scene
scenePath = reqMsg.SceneFiles;
sceneData = loadScene(scenePath);
if reqMsg.DoCalibration
    sceneData = loadCalib(reqMsg.CalibrationFiles,sceneData);
end

% Load scene point cloud
scenePointCloud = getScenePointCloud(sceneData);

% Load and align empty bin/tote point cloud to observation
binIds = 'ABCDEFGHIJKL';
if strcmp(sceneData.env,'shelf')
    backgroundPointCloud = emptyShelfModels{strfind(binIds,sceneData.binId)};
else
    backgroundPointCloud = emptyToteModel;
end
[tform,backgroundPointCloud] = pcregrigidGPU(backgroundPointCloud,scenePointCloud,'InlierRatio',0.8,'MaxIterations',200,'Tolerance',[0.0001,0.0009],'Verbose',false,'Extrapolate',true);
extBin2Bg = inv(tform.T');

if savePointCloudVis
  pcwrite(scenePointCloud,fullfile(visPath,'vis.pc.scene'),'PLYFormat','binary');
  pcwrite(backgroundPointCloud,fullfile(visPath,'vis.pc.bg'),'PLYFormat','binary');
end

% Parse object names
objList = sceneData.objects;
uniqueObjectList = unique(objList);
for objIdx = 1:length(uniqueObjectList)
    tic;
    objName = uniqueObjectList{objIdx};
    objNum = sum(ismember(objList,objName));
    objModel = objModels{find(ismember(objNames,objName))};
    if isempty(objModel)
        fprintf('Error: object model not loaded for %s!\n',objName);
    end

    % Do 6D pose estimation for each object and save the results
    objPoses = getObjectPose(scenePath,sceneData,scenePointCloud,backgroundPointCloud,extBin2Bg,objName,objModel,objNum);
%     objPoses = getObjectPose(tmpDataPath,visPath,totePosePath,data,backgroundPointCloud,extBin2Bg,objName,objModel,objNum,binId,savePointClouds,saveResultImages);
    for duplicateIdx = 1:length(objPoses)
        respMsg.Objects = [respMsg.Objects; objPoses(duplicateIdx)];
    end
    toc;
end
resp = true;
end
