function objectHypotheses = getObjectPose(scenePath,sceneData,scenePointCloud,backgroundPointCloud,extBin2Bg,objName,objModel,objNum)

global visPath;
global savePointCloudVis; 
global saveResultImageVis;
global emptyShelfModels;
global emptyToteModel;

% Parameters for both shelf and tote scenario
gridStep = 0.002; % grid size for downsampling point clouds
icpWorstRejRatio = 0.9; % ratio of outlier points to ignore during ICP
objectHypotheses = [];

% Parameters specific to shelf or tote scenario
if strcmp(sceneData.env,'tote')
  frames = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18];
  viewBounds = [-0.3, 0.3; -0.4, 0.4; -0.05, 0.2];
  pushBackAxis = [0; 0; -1];
else
  frames = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15];
  viewBounds = [-0.01, 0.40; -0.17, 0.17; -0.06, 0.20];
  pushBackAxis = [1; 0; -1];
end

% Parse segmentation masks and return confidence threshold used
[objMasks,segmThresh,segmConfMaps] = getObjectMasks(scenePath,objName,frames);

% Create segmented point cloud of object
[objSegmPts,objSegmConf] = getSegmentedPointCloud(sceneData,frames,objMasks,segmConfMaps);

% Handle objects without depth
sceneData.extBin2World = inv(sceneData.extBin2World);
if (strcmp(objName,'dasani_water_bottle') && binID ~= -1) || strcmp(objName,'rolodex_jumbo_pencil_cup') || strcmp(objName,'platinum_pets_dog_bowl')
  [finalRt,surfaceCentroid,surfaceRange] = getObjectPoseNoDepth(visPath,objSegmPts,objName,binID,frames,sceneData,objMasks,savePointClouds);
  finalRt = sceneData.extBin2World * finalRt;
  finalScore = mean(objSegmConf);
  for instanceIdx = 1:objNum
    if savePointClouds
      surfaceAxisPtsX = [surfaceRange(1,1):0.001:surfaceRange(1,2)];
      surfaceAxisPtsX = [surfaceAxisPtsX;repmat(surfaceCentroid(2),1,size(surfaceAxisPtsX,2));repmat(surfaceCentroid(3),1,size(surfaceAxisPtsX,2))];
      surfaceAxisPtsY = [surfaceRange(2,1):0.001:surfaceRange(2,2)];
      surfaceAxisPtsY = [repmat(surfaceCentroid(1),1,size(surfaceAxisPtsY,2));surfaceAxisPtsY;repmat(surfaceCentroid(3),1,size(surfaceAxisPtsY,2))];
      surfaceAxisPtsZ = [surfaceRange(3,1):0.001:surfaceRange(3,2)];
      surfaceAxisPtsZ = [repmat(surfaceCentroid(1),1,size(surfaceAxisPtsZ,2));repmat(surfaceCentroid(2),1,size(surfaceAxisPtsZ,2));surfaceAxisPtsZ];
      pcwrite(pointCloud([surfaceAxisPtsX,surfaceAxisPtsY,surfaceAxisPtsZ]','Color',[repmat([255,0,0],size(surfaceAxisPtsX,2),1);repmat([0,255,0],size(surfaceAxisPtsY,2),1);repmat([0,0,255],size(surfaceAxisPtsZ,2),1)]),fullfile(visPath,sprintf('vis.objSurf.%s.%d',objName,instanceIdx)),'PLYFormat','binary');
    end
    surfaceCentroidWorld = sceneData.extBin2World(1:3,1:3) * surfaceCentroid + repmat(sceneData.extBin2World(1:3,4),1,size(surfaceCentroid,2));
    surfaceRangeWorld = sceneData.extBin2World(1:3,1:3) * surfaceRange + repmat(sceneData.extBin2World(1:3,4),1,size(surfaceRange,2));
    if saveResultImages
      visualizeResults(finalRt,[0 0 0],surfaceCentroidWorld,surfaceRangeWorld,finalRt,finalScore,dataPath,objName,instanceIdx,binID,sceneData,[],objMasks,objModel.Location');
    end
    currObjectHypothesis = getObjectHypothesis(finalRt,[0 0 0],surfaceCentroidWorld,surfaceRangeWorld,finalRt,finalScore,dataPath,objName,instanceIdx);
    objectHypotheses = [objectHypotheses currObjectHypothesis];
  end
  return;
end

% Do 3D background subtraction
% pcwrite(pointCloud(objSegmPts'),fullfile(visPath,sprintf('vis.seg.%s',objName)),'PLYFormat','binary');
% pcwrite(backgroundPointCloud,fullfile(visPath,sprintf('vis.bg.%s',objName)),'PLYFormat','binary');
[indicesNN,distsNN] = multiQueryKNNSearchImplGPU(backgroundPointCloud,objSegmPts');
objSegmPts(:,find(sqrt(distsNN) < 0.005)) = [];
if strcmp(sceneData.env,'shelf')
  objSegmPtsBg = extBin2Bg(1:3,1:3) * objSegmPts + repmat(extBin2Bg(1:3,4) + [0;0;0.01],1,size(objSegmPts,2));
  bgRot = vrrotvec2mat([0 1 0 -atan(0.025/0.20)]);
  objSegmPtsBgRot = bgRot(1:3,1:3) * objSegmPtsBg;
%   pcwrite(pointCloud(objSegmPtsBgRot'),fullfile(visPath,sprintf('vis.objTestRot.%s',objName)),'PLYFormat','binary');
%   pcwrite(pointCloud(objSegmPts'),fullfile(visPath,sprintf('vis.objTestPre.%s',objName)),'PLYFormat','binary');
  objSegmPts(:,find(objSegmPtsBg(3,:) < -0.025 | objSegmPtsBgRot(3,:) < 0)) = [];
%   pcwrite(pointCloud(objSegmPts'),fullfile(visPath,sprintf('vis.objTestPost.%s',objName)),'PLYFormat','binary');
end

% Remove points outside the bin/tote
ptsOutsideBounds = find((objSegmPts(1,:) < viewBounds(1,1)) | (objSegmPts(1,:) > viewBounds(1,2)) | ...
                        (objSegmPts(2,:) < viewBounds(2,1)) | (objSegmPts(2,:) > viewBounds(2,2)) | ...
                        (objSegmPts(3,:) < viewBounds(3,1)) | (objSegmPts(3,:) > viewBounds(3,2)));
objSegmPts(:,ptsOutsideBounds) = [];
objSegmConf(:,ptsOutsideBounds) = [];

% % Grab center of segmented points and remove outliers
% obsCenter = median(sortrows(objSegPts',3)',2);
% eucDistGuess = sqrt(sum((objSegPts'-repmat(obsCenter',size(objSegPts,2),1)).^2,2));
% [sortDist, sortIdx] = sort(eucDistGuess);
% inlierPts = objCamPts(:,sortIdx(1:ceil((size(eucDistGuess,1) * icpWorstRejRatio))));

% If object not found, return dummy pose
if size(objSegmPts,2) < 200
  fprintf('%s: 0.000000\n',objName);
  for instanceIdx = 1:objNum
    currObjectHypothesis = getEmptyObjectHypothesis(scenePath,objName,instanceIdx);
    objectHypotheses = [objectHypotheses,currObjectHypothesis];
  end
  return;
end
  
% Do k-means clustering to find centroids per object instance (duplicates)
instancePts = {};
instanceConf = {};
[kIdx, kCtr] = kmeans(objSegmPts(1:2,:)',objNum);
for instanceIdx = 1:objNum
  instanceInd = find(kIdx==instanceIdx);
  instancePts{instanceIdx} = objSegmPts(:,instanceInd);
  instanceConf{instanceIdx} = objSegmConf(instanceInd);
end

% Load object model and downsample it
objModelPts = objModel.Location;
objModelCloud = pointCloud(objModelPts);
objModelCloud = pcdownsample(objModelCloud,'gridAverage',gridStep);

% Loop through each instance of the object and estimate its pose
for instanceIdx = 1:objNum
  
  % Load object model
  objModelPts = objModelCloud.Location';

  % Remove outliers from the segmented point cloud using PCA
  currObjSegPts = instancePts{instanceIdx};
  if ~strcmp(objName,'barkely_hide_bones')
    if saveResultImageVis
      pcwrite(pointCloud(currObjSegPts'),fullfile(visPath,sprintf('vis.objObs.%s.%d',objName,instanceIdx)),'PLYFormat','binary');
    end
    try
      currObjSegPts = denoisePointCloud(currObjSegPts);
    end
    if saveResultImageVis
      pcwrite(pointCloud(currObjSegPts'),fullfile(visPath,sprintf('vis.objCut.%s.%d',objName,instanceIdx)),'PLYFormat','binary');     
    end
  end
  fullCurrObjSegPts = currObjSegPts;
  
  % Print segmentation confidence
  finalScore = mean(instanceConf{instanceIdx});
  if strcmp(objName,'dasani_water_bottle') && binID == -1
    finalScore = finalScore/4;
  end
  if size(currObjSegPts,2) < 100 % If object not found
    fprintf('%s: 0.000000\n',objName);
    currObjectHypothesis = getEmptyObjectHypothesis(dataPath,objName,instanceIdx);
    objectHypotheses = [objectHypotheses currObjectHypothesis];
    continue;
  else
    fprintf('%s: %f\n',objName,finalScore);
  end
    
  % Downsample segmented point cloud to same resolution as object model
  currObjSegCloud = pointCloud(currObjSegPts');
  currObjSegCloud = pcdownsample(currObjSegCloud,'gridAverage',gridStep);
  currObjSegCloud = pcdenoise(currObjSegCloud,'NumNeighbors',4);
  
  if size(currObjSegCloud.Location',2) < 200 % If object not found
    currObjectHypothesis = getEmptyObjectHypothesis(dataPath,objName,instanceIdx);
    objectHypotheses = [objectHypotheses currObjectHypothesis];
    continue;
  end
  
  % Compute surface mean 
  surfaceCentroid = mean(currObjSegPts,2);
  currObjSegPtsRangeX = currObjSegPts(:,find(((currObjSegPts(2,:)>(surfaceCentroid(2)-0.005)) & (currObjSegPts(2,:)<(surfaceCentroid(2)+0.005)) & ...
                                              (currObjSegPts(3,:)>(surfaceCentroid(3)-0.005)) & (currObjSegPts(3,:)<(surfaceCentroid(3)+0.005)))));
  currObjSegPtsRangeY = currObjSegPts(:,find(((currObjSegPts(1,:)>(surfaceCentroid(1)-0.005)) & (currObjSegPts(1,:)<(surfaceCentroid(1)+0.005)) & ...
                                              (currObjSegPts(3,:)>(surfaceCentroid(3)-0.005)) & (currObjSegPts(3,:)<(surfaceCentroid(3)+0.005)))));
  currObjSegPtsRangeZ = currObjSegPts(:,find(((currObjSegPts(2,:)>(surfaceCentroid(2)-0.005)) & (currObjSegPts(2,:)<(surfaceCentroid(2)+0.005)) & ...
                                              (currObjSegPts(1,:)>(surfaceCentroid(1)-0.005)) & (currObjSegPts(1,:)<(surfaceCentroid(1)+0.005)))));
  if isempty(currObjSegPtsRangeX)
    currObjSegPtsRangeX = currObjSegPts;
  end
  if isempty(currObjSegPtsRangeY)
    currObjSegPtsRangeY = currObjSegPts;
  end
  if isempty(currObjSegPtsRangeZ)
    currObjSegPtsRangeZ = currObjSegPts;
  end
  surfaceRangeX = [min(currObjSegPtsRangeX(1,:)),max(currObjSegPtsRangeX(1,:))];
  surfaceRangeY = [min(currObjSegPtsRangeY(2,:)),max(currObjSegPtsRangeY(2,:))];
  surfaceRangeZ = [min(currObjSegPtsRangeZ(3,:)),max(currObjSegPtsRangeZ(3,:))];
  surfaceRange = [surfaceRangeX;surfaceRangeY;surfaceRangeZ];
                                            
  % Visualize surface centroid and surface ranges
  if savePointCloudVis
    surfaceAxisPtsX = [surfaceRangeX(1):0.001:surfaceRangeX(2)];
    surfaceAxisPtsX = [surfaceAxisPtsX;repmat(surfaceCentroid(2),1,size(surfaceAxisPtsX,2));repmat(surfaceCentroid(3),1,size(surfaceAxisPtsX,2))];
    surfaceAxisPtsY = [surfaceRangeY(1):0.001:surfaceRangeY(2)];
    surfaceAxisPtsY = [repmat(surfaceCentroid(1),1,size(surfaceAxisPtsY,2));surfaceAxisPtsY;repmat(surfaceCentroid(3),1,size(surfaceAxisPtsY,2))];
    surfaceAxisPtsZ = [surfaceRangeZ(1):0.001:surfaceRangeZ(2)];
    surfaceAxisPtsZ = [repmat(surfaceCentroid(1),1,size(surfaceAxisPtsZ,2));repmat(surfaceCentroid(2),1,size(surfaceAxisPtsZ,2));surfaceAxisPtsZ];
    pcwrite(pointCloud([surfaceAxisPtsX,surfaceAxisPtsY,surfaceAxisPtsZ]','Color',[repmat([255,0,0],size(surfaceAxisPtsX,2),1);repmat([0,255,0],size(surfaceAxisPtsY,2),1);repmat([0,0,255],size(surfaceAxisPtsZ,2),1)]),fullfile(visPath,sprintf('vis.objSurf.%s.%d',objName,instanceIdx)),'PLYFormat','binary');
  end
  
  % Convert surface centroid to world coordinates
  surfaceCentroid = sceneData.extBin2World(1:3,1:3) * surfaceCentroid + repmat(sceneData.extBin2World(1:3,4),1,size(surfaceCentroid,2));
  surfaceRange = sceneData.extBin2World(1:3,1:3) * surfaceRange + repmat(sceneData.extBin2World(1:3,4),1,size(surfaceRange,2));
  
  % Recompute PCA Rt over denoised and downsampled segmented point cloud
  currObjSegPts = currObjSegCloud.Location';
  [coeffPCA,scorePCA,latentPCA] = pca(currObjSegPts');
  if size(latentPCA,1) < 3
    latentPCA = [latentPCA;0];
  end
  coeffPCA = [coeffPCA(:,1),coeffPCA(:,2),cross(coeffPCA(:,1),coeffPCA(:,2))]; % Follow righthand rule
  initRtPCA = [[coeffPCA median(currObjSegPts,2)]; 0 0 0 1];
%   testPCA = [[coeffPCAoutlier median(currObjSegPts,2)]; 0 0 0 1];
  if savePointCloudVis
    axisPts = [[[0:0.001:latentPCA(1)*50]; zeros(2,size([0:0.001:latentPCA(1)*50],2))],[zeros(1,size([0:0.001:latentPCA(2)*50],2)); [0:0.001:latentPCA(2)*50]; zeros(1,size([0:0.001:latentPCA(2)*50],2))],[zeros(2,size([0:0.001:latentPCA(3)*50],2)); [0:0.001:latentPCA(3)*50]]];
    axisColors = uint8([repmat([255; 0; 0],1,size([0:0.001:latentPCA(1)*50],2)),repmat([0; 255; 0],1,size([0:0.001:latentPCA(2)*50],2)),repmat([0; 0; 255],1,size([0:0.001:latentPCA(3)*50],2))]);
    tmpAxisPts = initRtPCA(1:3,1:3) * axisPts + repmat(initRtPCA(1:3,4),1,size(axisPts,2));
    pcwrite(pointCloud(tmpAxisPts','Color',axisColors'),fullfile(visPath,sprintf('vis.objPost.%s.%d',objName,instanceIdx)),'PLYFormat','binary');
  end
  RtPCA = sceneData.extBin2World * initRtPCA;
  
  % If object is deformable, return PCA as pose
  if strcmp(objName,'cherokee_easy_tee_shirt') || strcmp(objName,'kyjen_squeakin_eggs_plush_puppies') || strcmp(objName,'womens_knit_gloves') || strcmp(objName,'cloud_b_plush_bear')
    finalRt = RtPCA;
    if saveResultImageVis
      visualizeResults(RtPCA,latentPCA,surfaceCentroid,surfaceRange,finalRt,finalScore,dataPath,objName,instanceIdx,binID,sceneData,fullCurrObjSegPts,objMasks,objModel.Location');
    end
    currObjectHypothesis = getObjectHypothesis(RtPCA,latentPCA,surfaceCentroid,surfaceRange,finalRt,finalScore,dataPath,objName,instanceIdx);
    objectHypotheses = [objectHypotheses currObjectHypothesis];
    continue;
  end

  % Push objects back prior to ICP
  pushBackVal = max([objModel.XLimits(2),objModel.YLimits(2),objModel.ZLimits(2)]);
  initRtPCA(1:3,4) = initRtPCA(1:3,4) + pushBackAxis.*pushBackVal;
  
  % Attempt 6 different rotations using PCA directions
%   possibleRotations = cell(1,6);
%   possibleRotations{1} = eye(3);
%   possibleRotations{2} = vrrotvec2mat([1,0,0,pi/2]);
%   possibleRotations{3} = vrrotvec2mat([1,0,0,pi]);
%   possibleRotations{4} = vrrotvec2mat([1,0,0,3*pi/2]);
%   possibleRotations{5} = vrrotvec2mat([0 1 0 pi/2]);
%   possibleRotations{6} = vrrotvec2mat([0,1,0,3*pi/2]);
  bestRotScore = inf;
  bestRt = initRtPCA;
%   for rotIdx = 1:length(possibleRotations)
%     tmpRt = initRtPCA * [possibleRotations{rotIdx} [0;0;0]; [0,0,0,1]];
%     tmpObjModelPts = tmpRt(1:3,1:3) * objModelPts + repmat(tmpRt(1:3,4),1,size(objModelPts,2));
%     tmpObjModelCloud = pointCloud(tmpObjModelPts');
% %     tmpObjSegCloud = pointCloud(currObjSegPts');
% %     [tform,movingReg,rmse] = pcregrigid(tmpObjSegCloud,tmpObjModelCloud,'InlierRatio',0.8,'MaxIterations',1);
%     [tmpIndices,tmpDists1] = multiQueryKNNSearchImplGPU(tmpObjModelCloud,currObjSegPts');
%     tmpObjSegCloud = pointCloud(currObjSegPts');
%     [tmpIndices,tmpDists2] = multiQueryKNNSearchImplGPU(tmpObjSegCloud,tmpObjModelPts');
%     tmpScore = mean([tmpDists1,tmpDists2]);
%     if tmpScore < bestRotScore
%       bestRotScore = tmpScore;
%       bestRt = tmpRt;
%     end
%   end
%   initRtPCA = bestRt;
  if strcmp(sceneData.env,'shelf') && (strcmp(objName,'creativity_chenille_stems') || strcmp(objName,'dr_browns_bottle_brush') || strcmp(objName,'peva_shower_curtain_liner') || strcmp(objName,'kleenex_paper_towels'))
    initRtPCA(1:3,1:3) = eye(3);
  end
  
  % Apply rigid transform computed prior to ICP
  tmpObjModelPts = initRtPCA(1:3,1:3) * objModelPts + repmat(initRtPCA(1:3,4),1,size(objModelPts,2));
  if savePointCloudVis
    pcwrite(pointCloud(tmpObjModelPts','Color',repmat(uint8([0 0 255]),size(tmpObjModelPts,2),1)),fullfile(visPath,sprintf('vis.objPre.%s.%d',objName,instanceIdx)),'PLYFormat','binary');
  end
  
  % Perform ICP to align segmented point cloud to object model
  if size(currObjSegPts,2) > 3
    tmpObjModelCloud = pointCloud(tmpObjModelPts');
    objSegCloud = pointCloud(currObjSegPts');
    [tform,movingReg,icpRmse] = pcregrigidGPU(objSegCloud,tmpObjModelCloud,'InlierRatio',icpWorstRejRatio,'MaxIterations',200,'Tolerance',[0.0001 0.0009],'Verbose',false,'Extrapolate',true);
    icpRt1 = inv(tform.T');
    tmpObjModelPts = tmpObjModelCloud.Location';
    tmpObjModelPts = icpRt1(1:3,1:3) * tmpObjModelPts + repmat(icpRt1(1:3,4),1,size(tmpObjModelPts,2));
    tmpObjModelCloud = pointCloud(tmpObjModelPts');
    [tform,movingReg,icpRmse] = pcregrigidGPU(objSegCloud,tmpObjModelCloud,'InlierRatio',icpWorstRejRatio/2,'MaxIterations',200,'Tolerance',[0.0001 0.0009],'Verbose',false,'Extrapolate',true);
    icpRt2 = inv(tform.T');
    icpRt = icpRt2*icpRt1;
  else
    icpRt = eye(4);
  end
  finalRt = icpRt * initRtPCA;
  
  tmpObjModelPts = finalRt(1:3,1:3) * objModelPts + repmat(finalRt(1:3,4),1,size(objModelPts,2));
  if savePointCloudVis
    pcwrite(pointCloud(tmpObjModelPts','Color',repmat(uint8([0 255 0]),size(tmpObjModelPts,2),1)),fullfile(visPath,sprintf('vis.objPost.%s.%d',objName,instanceIdx)),'PLYFormat','binary');
  end
  
%   plot3(bgCloud.Location(:,1),bgCloud.Location(:,2),bgCloud.Location(:,3),'.r');
%   hold on; axis equal; grid on; xlabel('x'); ylabel('y'); zlabel('z'); hold off;
%   pcshow(bgCloud)

  % Save pose as a rigid transform from model to world space
  
%   tmpAxisPts = finalRt(1:3,1:3) * axisPts + repmat(finalRt(1:3,4),1,size(axisPts,2));
%   pcwrite(pointCloud(tmpAxisPts','Color',axisColors'),fullfile(visPath,sprintf('vis.objAxis.%s.%d',objName,instanceIdx)),'PLYFormat','binary');
  
%   if (finalRt(1) < viewBounds(1,1)) || (finalRt(1) > viewBounds(1,2)) || ...
%      (finalRt(2) < viewBounds(2,1)) || (finalRt(2) > viewBounds(2,2)) || ...
%      (finalRt(3) < viewBounds(3,1)) || (finalRt(3) > viewBounds(3,2))
%     currObjectHypothesis = getEmptyObjectHypothesis(dataPath,objName,instanceIdx);
%     objectHypotheses = [objectHypotheses currObjectHypothesis];
%     continue;
%   end
  
  finalRt = sceneData.extBin2World * finalRt;
  if saveResultImageVis
    visualizeResults(RtPCA,latentPCA,surfaceCentroid,surfaceRange,finalRt,finalScore,scenePath,objName,instanceIdx,sceneData,fullCurrObjSegPts,objMasks,objModel.Location');
  end
  currObjectHypothesis = getObjectHypothesis(RtPCA,latentPCA,surfaceCentroid,surfaceRange,finalRt,finalScore,scenePath,objName,instanceIdx);
  objectHypotheses = [objectHypotheses,currObjectHypothesis];
end

end

