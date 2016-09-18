function sceneData = loadCalib(calibDir,sceneData)
% Load relative camera-to-camera poses generated from getCalib and use them
% to fix the camera poses for the captured RGB-D sequence saved in sceneData
%
% function sceneData = loadCalib(calibDir,sceneData)
% Input:
%   calibDir  - file path to the folder that contains the set of relative 
%               camera-to-camera poses generated from getCalib
%   sceneData - data structure holding the contents (frames and camera
%               information) of a captured RGB-D scene
% Output:
%   sceneData - scene data with calibrated camera-to-world matrices
%
% ---------------------------------------------------------
% Copyright (c) 2016, Andy Zeng
% 
% This file is part of the APC Vision Toolbox and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------

% Retrieve pivot viewpoint camera pose based on the sequence environment
if strcmp(sceneData.env,'tote')
    baseFrameIdx = 1;
    calibPoseFile = fullfile(calibDir,'tote','cam.poses.txt');
else
    baseFrameIdx = 8;
    calibPoseFile = fullfile(calibDir,'shelf',sprintf('cam.poses.%s.txt',sceneData.binId));
end
extPivot2World = sceneData.extCam2World{baseFrameIdx};

% Read camera poses
extCam2Pivot = {};
calibPoseFileInd = 1;
while true
    try
        extCam2Pivot{length(extCam2Pivot)+1} = dlmread(calibPoseFile,'\t',[calibPoseFileInd,0,calibPoseFileInd+3,3]);
        calibPoseFileInd = calibPoseFileInd+6;
    catch
        break;
    end
end

% Apply calibration camera poses
for frameIdx = 1:length(sceneData.colorFrames)
    sceneData.extCam2World{frameIdx} = extPivot2World * extCam2Pivot{frameIdx};
end
