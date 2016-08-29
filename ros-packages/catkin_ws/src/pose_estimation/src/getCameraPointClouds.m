function [data,bgCloud,camCloud,bin2BgRt] = getCameraPointClouds(dataPath,totePosePath,shelfPosePath,visPath,binID,savePointClouds)

% Rviz parameters
rvizPC = false; global vizpub;

% Parameters specific to shelf or tote scenario
if binID == -1
  frames = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18];
  camPosePath = totePosePath;
else
  frames = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15];
  camPosePath = shelfPosePath;
end

% Get RGB-D frame data and camera poses
data = getData(dataPath,camPosePath,frames,binID);

% Create aggregated RGB-D point cloud (15 frames)
for frameIdx = frames
    color = data.color{frameIdx};
    depth = data.depth{frameIdx};
    extCam2Bin = data.extWorld2Bin * data.extCam2World{frameIdx};

    % Project depth into camera space
    [pixX,pixY] = meshgrid(1:640,1:480);
    camX = (pixX-data.K(1,3)).*depth/data.K(1,1);
    camY = (pixY-data.K(2,3)).*depth/data.K(2,2);
    camZ = depth;
    
    % Only use points with valid depth
    validDepth = find((camZ > 0) & (camZ < 1));
    camPts = [camX(validDepth),camY(validDepth),camZ(validDepth)]';
    camPts = extCam2Bin(1:3,1:3) * camPts + repmat(extCam2Bin(1:3,4),1,size(camPts,2));
    
    % Get vertex colors
    colorR = color(:,:,1);
    colorG = color(:,:,2);
    colorB = color(:,:,3);
    colorPts = [colorR(validDepth),colorG(validDepth),colorB(validDepth)]';
    
    % Aggregate point clouds
    if frameIdx == frames(1)
      camCloud = pointCloud(camPts','Color',colorPts');
    else
      camCloud = pcmerge(camCloud,pointCloud(camPts','Color',colorPts'),0.002);
    end
end

if savePointClouds
  pcwrite(camCloud,fullfile(visPath,'vis.camCloud'),'PLYFormat','binary');
end
if rvizPC
  send(vizpub, createPointCloudMarker(sprintf('vis.camCloud'), allCamPts, [1 1 1], allColorPts, sprintf('bin%d', binID), 100));
end

% Save denoised and downsampled point cloud for planning
% if savePointClouds
%   sparseCamCloud = pcdenoise(camCloud,'NumNeighbors',4*15);
%   sparseCamCloud = pcdownsample(sparseCamCloud,'gridAverage',0.0025);
%   if binID == -1
%     viewBounds = [-0.3, 0.3; -0.4, 0.4; -0.05, 0.2];
%   else
%     viewBounds = [-0.10, 0.40; -0.17, 0.17; -0.06, 0.20];
%   end
%   sparseCamPts = sparseCamCloud.Location';
%   sparseCamColors = sparseCamCloud.Color';
%   ptsOutsideBounds = find((sparseCamPts(1,:) < viewBounds(1,1)) | (sparseCamPts(1,:) > viewBounds(1,2)) | ...
%                           (sparseCamPts(2,:) < viewBounds(2,1)) | (sparseCamPts(2,:) > viewBounds(2,2)) | ...
%                           (sparseCamPts(3,:) < viewBounds(3,1)) | (sparseCamPts(3,:) > viewBounds(3,2)));
%   sparseCamPts(:,ptsOutsideBounds) = [];
%   sparseCamColors(:,ptsOutsideBounds) = [];
%   sparseCamCloud = pointCloud(sparseCamPts','Color',sparseCamColors');
%   pcwrite(sparseCamCloud,fullfile(visPath,'vis.sparse'),'PLYFormat','binary');
% end

% plot3(allCamPts(1,:)',allCamPts(2,:)',allCamPts(3,:)','.r');
% hold on; axis equal; grid on; xlabel('x'); ylabel('y'); zlabel('z'); hold off;

% Align empty bin to observation
bgCloud = pcread(sprintf('bins/bin%d.ply',binID));
[tform,bgCloud] = pcregrigidGPU(bgCloud,camCloud,'InlierRatio',0.8,'MaxIterations',200,'Tolerance',[0.0001 0.0009],'Verbose',false,'Extrapolate',true);
bin2BgRt = inv(tform.T');

if savePointClouds
  pcwrite(bgCloud,fullfile(visPath,'vis.bgCloud'),'PLYFormat','binary');
end

end

