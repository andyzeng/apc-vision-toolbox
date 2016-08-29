function getCalib(calibDir,calibPoseFile)
% Generate a better set of camera poses for a sequence of RGB-D frames. 
% This calibration step assumes that consistent relative camera-to-camera
% poses can be achieved with the robot. 
%
% Summary: Pick one frame and set its original camera-to-world camera pose 
% as the pivot pose. Use Structure from Motion to generate relative 
% camera-to-camera poses, then use the pivot pose to generate the new
% camera-to-world camera poses for all views. Note that the error from the
% pivot pose propagates to the estimated camera poses of the other views.
%
% function getCalib(calibDir,calibPoseFile)
% Input:
%   calibDir      - file path to the directory containing a calibration
%                   sequence of RGB-D frames (these captured frames should
%                   be rich with 2D color features)
%   calibPoseFile - file path to the file in which the new set of
%                   camera-to-world camera poses will be saved
%
% ---------------------------------------------------------
% Copyright (c) 2016, Andy Zeng
% 
% This file is part of the APC Vision Toolbox and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------

% Load external dependencies
addpath(genpath('external'));

% Load RGB-D sequence and camera information
sceneData = loadScene(calibDir);

% Compute SURF features from each RGB-D frame
numFrames = length(sceneData.colorFrames);
sceneData.SURFPts3D = cell(1,numFrames);
sceneData.SURFDesc = cell(1,numFrames);
for frameIdx = 1:numFrames
    
    % Extract SURF features from the image
    grayFrame = rgb2gray(sceneData.colorFrames{frameIdx});
    [SURFDesc,SURFFeat] = extractFeatures(grayFrame,detectSURFFeatures(grayFrame));
    
    % Project SURF points into 3D camera space
    SURFPts2D = zeros(2,length(SURFFeat));
    SURFPtsDepth = zeros(1,length(SURFFeat));
    depthFrame = sceneData.depthFrames{frameIdx};
    for SURFIdx = 1:length(SURFFeat)
        SURFPts2D(:,SURFIdx) = round(SURFFeat(SURFIdx).Location');
        SURFPtsDepth(SURFIdx) = depthFrame(SURFPts2D(2,SURFIdx),SURFPts2D(1,SURFIdx));
    end
    SURFPtsX = (SURFPts2D(1,:)-sceneData.colorK(1,3)).*SURFPtsDepth/sceneData.colorK(1,1);
    SURFPtsY = (SURFPts2D(2,:)-sceneData.colorK(2,3)).*SURFPtsDepth/sceneData.colorK(2,2);
    SURFPtsZ = SURFPtsDepth;
    SURFPts3D = [SURFPtsX;SURFPtsY;SURFPtsZ];

    % Only use SURF points with valid depth
    validDepth = find(SURFPts3D(3,:)>0.15);
    sceneData.SURFPts3D{frameIdx} = SURFPts3D(:,validDepth);
    sceneData.SURFDesc{frameIdx} = SURFDesc(validDepth,:);
end

% A mapping from one frame to its nearest frame in the sequence (these
% mappings are predefined for shelf and tote; if a frame is mapped to 0,
% its camera-to-world extrinsic matrix is set as the pivot transformation)
if strcmp(sceneData.env,'tote')
    nearestFrameIdx = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18;
                       0,1,2,1,4,5,4,7,8, 7,10,11,10,13,14,13,16,17];  
else
    nearestFrameIdx = [8,7,6,9,10,3,2,1,4,5,13,12,11,14,15;
                       0,8,7,8, 9,8,3,2,3,4, 8,13,12,13,14];  
end
               
% Use RANSAC to estimate the rigid transformation between each camera view               
newExtCam2World = cell(1,numFrames);
for frameIdx = 1:numFrames
    
    % Set pivot camera pose
    if nearestFrameIdx(2,frameIdx) == 0
        newExtCam2World{nearestFrameIdx(1,frameIdx)} = eye(4);
%         newExtCam2World{nearestFrameIdx(1,frameIdx)} = seqData.extCam2World{nearestFrameIdx(1,frameIdx)};
        continue;
    end
    
    % Get SURF features of the current frame and nearest frame
    currentSURFpts3D = sceneData.SURFPts3D{nearestFrameIdx(1,frameIdx)};
    currentFrameDesc = sceneData.SURFDesc{nearestFrameIdx(1,frameIdx)};
    nearestSURFpts3D = sceneData.SURFPts3D{nearestFrameIdx(2,frameIdx)};
    nearestFrameDesc = sceneData.SURFDesc{nearestFrameIdx(2,frameIdx)};
    
    % Match SURF features
    SURFMatchInd = matchFeatures(nearestFrameDesc,currentFrameDesc);
    nearestSURFpts3D  = nearestSURFpts3D(:,SURFMatchInd(:,1));
    currentSURFpts3D = currentSURFpts3D(:,SURFMatchInd(:,2));

    % Use RANSAC to estimate rigid transformation between the two frames
    [initRt,ransacInliers] = ransacfitRt([nearestSURFpts3D;currentSURFpts3D], 0.005,0);
    % fprintf('Aligning frame-%06d to frame-%06d. RANSAC inliers: %d/%d\n',nearestFrameIdx(1,frameIdx)-1,nearestFrameIdx(2,frameIdx)-1,size(ransacInliers,2),size(currentSURFpts3D,2));
    
    newExtCam2World{nearestFrameIdx(1,frameIdx)} = newExtCam2World{nearestFrameIdx(2,frameIdx)} * [initRt; [0 0 0 1]];
end

% Save new camera-to-world extrinsics to file
calibPoseFileId = fopen(calibPoseFile,'wb');
for frameIdx = 1:numFrames
    fprintf(calibPoseFileId,'# Camera-to-camera extrinsic matrix (camera pose) from frame-%06d to frame-%06d\n',frameIdx-1,nearestFrameIdx(1)-1);
    fprintf(calibPoseFileId,'%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n',newExtCam2World{frameIdx}');
    fprintf(calibPoseFileId,'\n');
end
fclose(calibPoseFileId);