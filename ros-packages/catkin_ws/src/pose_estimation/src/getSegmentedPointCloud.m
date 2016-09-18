function [objSegmPts,objSegmConf] = getSegmentedPointCloud(sceneData,frames,objMasks,segConfMaps)
% Take in scene RGB-D data and segmentation confidence maps of a particular
% object to return a 3D point cloud of the segmented object
%
% function objSegPts = denoisePointCloud(objSegPts)
% Input:
%   sceneData   - data structure holding the contents (frames and camera
%                 information) of a captured scene (has N RGB-D frames)
%   frames      - 1xN array indicating which frames of the RGB-D sequence
%                 from sceneData to use
%   objMasks    - 1xN cell array of 480x640 binary object masks (computed
%                 from segmentation)
%   segConfMaps - 1xN cell array of 480x640 confidence values from
%                 segmentation
% Output:
%   objSegmPts  - 3xK float array of K 3D points of the segmented object
%   objSegmConf - 1xK float array of confidence values for each of the K 
%                 points of the segmented object
%
% ---------------------------------------------------------
% Copyright (c) 2016, Andy Zeng
% 
% This file is part of the APC Vision Toolbox and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------

% Create segmented object point cloud 
objSegmPts = [];
objSegmConf = [];
for frameIdx = frames
    tmpObjMask = objMasks{frameIdx};
    tmpSegConfMap = segConfMaps{frameIdx};
    tmpDepth = sceneData.depthFrames{frameIdx};
    tmpExtCam2World = sceneData.extCam2World{frameIdx};
    tmpExtCam2Bin = sceneData.extWorld2Bin * tmpExtCam2World;
    
    % Apply segmentation mask to depth image and project to camera space
    tmpDepth = tmpDepth.*double(tmpObjMask);
    [pixX,pixY] = meshgrid(1:640,1:480);
    camX = (pixX-sceneData.colorK(1,3)).*tmpDepth/sceneData.colorK(1,1);
    camY = (pixY-sceneData.colorK(2,3)).*tmpDepth/sceneData.colorK(2,2);
    camZ = tmpDepth;
    validDepth = find((camZ > 0.1) & (camZ < 1));
    camPts = [camX(validDepth),camY(validDepth),camZ(validDepth)]';
    camPts = tmpExtCam2Bin(1:3,1:3) * camPts + repmat(tmpExtCam2Bin(1:3,4),1,size(camPts,2));
    objSegmPts = [objSegmPts,camPts];
    objSegmConf = [objSegmConf,tmpSegConfMap(validDepth)'];
end

end

