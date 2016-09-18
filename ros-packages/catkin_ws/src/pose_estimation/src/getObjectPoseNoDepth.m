function [predObjPoseBin,surfCentroid,surfRange] = getObjectPoseNoDepth(visPath,objSegmPts,objName,frames,sceneData,objMasks)
% Take in scene RGB-D data and segmentation confidence maps of a particular
% object (with no depth) and predict a 6D pose using triangulation
%
% function [predObjPoseBin,surfCentroid,surfRange] = getObjectPoseNoDepth(visPath,objSegmPts,objName,frames,sceneData,objMasks)
% Input:
%   visPath     - directory to save point cloud visualization files
%   objSegmPts  - 3xK float array of K 3D points of the segmented object
%   objName     - name of the target object (aka. object ID)
%   frames      - 1xN array indicating which frames of the RGB-D sequence
%                 from sceneData to use
%   sceneData   - data structure holding the contents (frames and camera
%                 information) of a captured scene with N RGB-D frames
%   objMasks    - 1xN cell array of 480x640 binary object masks (computed
%                 from segmentation)
%   segConfMaps - 1xN cell array of 480x640 confidence values from
%                 segmentation
% Output:
%   predObjPoseBin - 4x4 predicted object pose in bin's coordinate system
%                    object-to-bin (homogenous coordinates)
%   surfCentroid   - predicted centroid of the object
%   surfRange      - predicted object 3D bounding box
%
% ---------------------------------------------------------
% Copyright (c) 2016, Andy Zeng
% 
% This file is part of the APC Vision Toolbox and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------

global savePointCloudVis;

% Discretize bin into 3D grid
if strcmp(sceneData.env,'shelf')
  [binGridX,binGridY,binGridZ] = ndgrid(-0.05:0.01:0.40,-0.14:0.01:0.17,-0.04:0.01:0.20);
  gridOccupancyThreshold = floor(0.5*length(frames));
else
  [binGridX,binGridY,binGridZ] = ndgrid(-0.16:0.01:0.16,-0.23:0.01:0.32,-0.03:0.01:0.10);
  gridOccupancyThreshold = floor(0.3*length(frames));
end
binGridPts = [binGridX(:),binGridY(:),binGridZ(:)]';

% Compute grid occupancy from 2D segmentation results
binGridOccupancy = zeros(size(binGridX(:)));
for frameIdx = frames
  currObjMask = objMasks{frameIdx};
  binGridPtsWorld = sceneData.extBin2World(1:3,1:3) * binGridPts + repmat(sceneData.extBin2World(1:3,4),1,size(binGridPts,2));
  currExtWorld2Cam = inv(sceneData.extCam2World{frameIdx});
  binGridPtsCam = currExtWorld2Cam(1:3,1:3) * binGridPtsWorld + repmat(currExtWorld2Cam(1:3,4),1,size(binGridPtsWorld,2));
  binGridPtsPix = binGridPtsCam;
  binGridPtsPix(1,:) = round((binGridPtsPix(1,:).*sceneData.colorK(1,1))./binGridPtsPix(3,:)+sceneData.colorK(1,3));
  binGridPtsPix(2,:) = round((binGridPtsPix(2,:).*sceneData.colorK(2,2))./binGridPtsPix(3,:)+sceneData.colorK(2,3));
  pixWithinImage = find((binGridPtsPix(1,:) <= 640) & (binGridPtsPix(1,:) > 0) & (binGridPtsPix(2,:) <= 480) & (binGridPtsPix(2,:) > 0));
  occupiedBinGridInd = pixWithinImage(find(currObjMask(sub2ind(size(currObjMask),binGridPtsPix(2,pixWithinImage)',binGridPtsPix(1,pixWithinImage)')) > 0));
  binGridOccupancy(occupiedBinGridInd) = binGridOccupancy(occupiedBinGridInd) + 1;
end

predObjPoseBin = eye(4);

bestGuessPtsInd = find(binGridOccupancy > gridOccupancyThreshold);
if length(bestGuessPtsInd) < 10
  bestGuessPtsInd = find(binGridOccupancy > 0);
end
objPts = binGridPts(:,bestGuessPtsInd);
instanceIdx = 1;
objPts = sortrows(objPts',1);
objCentroid = median(objPts);
if ~isempty(objPts)
    predObjPoseBin(1:3,4) = objCentroid';
end

if strcmp(sceneData.env,'shelf')
  if strcmp(objName,'dasani_water_bottle')
    if objCentroid(1) < 0.20 % If on incline
      if objCentroid(3) + 0.03 > 0.06
        predObjPoseBin(1:3,1:3) = [0 0 1; -1 0 0; 0 -1 0]';
        fprintf('Standing up on front\n');
      else
          predObjPoseBin(1:3,1:3) = [-1 0 0; 0 0 1; 0 1 0]';
        fprintf('Lying pointing front on front\n');
      end
    else
      if objCentroid(3) + 0.04 > 0.06
        predObjPoseBin(1:3,1:3) = [0 0 1; -1 0 0; 0 -1 0]';
        fprintf('Standing up on back\n');
      else
        ptsAlongCentroidX = find(objPts(:,1) == max(objPts(:,1)));
        if max(objPts(ptsAlongCentroidX,2))-min(objPts(ptsAlongCentroidX,2)) > 0.10
          predObjPoseBin(1:3,1:3) = [0 -1 0; 0 0 1; -1 0 0]';
          fprintf('Lying on back sideways\n');
        else
          predObjPoseBin(1:3,1:3) = [-1 0 0; 0 0 1; 0 1 0]';
          fprintf('Lying on back pointing forward\n');
        end
      end
    end
  else

  end
  
else
  if strcmp(objName,'rolodex_jumbo_pencil_cup')
    objCamPts = objSegmPts;
    objCamPts = objCamPts(:,find((abs(objCamPts(1,:) - objCentroid(1)) < 0.04) & (abs(objCamPts(2,:) - objCentroid(2)) < 0.04)));
    numPtsAroundCentroid = size(objCamPts,2);
    objCamPts = objCamPts(:,find(objCamPts(3,:) > 0.08 & objCamPts(3,:) < 0.15));
    ptsElevatedRatio = size(objCamPts,2)/numPtsAroundCentroid;
    if ptsElevatedRatio > 0.5
      topPts = mean(objCamPts,2);
      predObjPoseBin(1:3,1:3) = [0 0 -1; -1 0 0; 0 -1 0]';
    end
  end
end

surfCentroid = predObjPoseBin(1:3,4);
if strcmp(objName,'rolodex_jumbo_pencil_cup') && strcmp(sceneData.env,'tote') && exist('topPts','var')
  surfRange = [surfCentroid(1)-0.05,surfCentroid(1)+0.05;surfCentroid(2)-0.05,surfCentroid(2)+0.05;surfCentroid(3)-0.05,topPts(3)];
else
  surfRange = [surfCentroid(1)-0.05,surfCentroid(1)+0.05;surfCentroid(2)-0.05,surfCentroid(2)+0.05;surfCentroid(3)-0.05,surfCentroid(3)+0.05];
end

  
% Enforce dog bowl (which has no depth) to always point upward
if strcmp(objName,'platinum_pets_dog_bowl')
  predObjPoseBin(1:3,1:3) = [-1 0 0; 0 1 0; 0 0 -1];
end

if savePointCloudVis
  pcwrite(pointCloud(objPts,'Color',repmat(uint8([0 255 0]),size(objPts,1),1)),fullfile(visPath,sprintf('vis.objDots.%s.%d',objName,instanceIdx)),'PLYFormat','binary');
  axisPts = [[[0:0.001:0.099]; zeros(2,100)],[zeros(1,100); [0:0.001:0.099]; zeros(1,100)],[zeros(2,100); [0:0.001:0.099]]];
  axisColors = uint8([repmat([255; 0; 0],1,100),repmat([0; 255; 0],1,100),repmat([0; 0; 255],1,100)]);
  tmpAxisPts = predObjPoseBin(1:3,1:3) * axisPts + repmat(predObjPoseBin(1:3,4),1,size(axisPts,2));
%   pcwrite(pointCloud(tmpAxisPts','Color',axisColors'),fullfile(visPath,sprintf('vis.objPost.%s.%d',objName,instanceIdx)),'PLYFormat','binary');
end

end

