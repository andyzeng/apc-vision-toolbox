function extCam2World = loadCalib(calibPoseFile)
%LOADCALIB Load the camera-to-world camera poses generated from calibSeq.
%
% Arguments: (input)
%   calibPoseFile - file path to the file that contains the new set of
%                   camera-to-world camera poses generated from calibSeq
%
% Arguments: (output)
%   extCam2World  - 1xn cell array of 4x4 camera-to-world extrinsic
%                   matrices (camera pose, in homogeneous coordinates)
%
% Author: Andy Zeng, andyz@princeton.edu

% Read camera poses
extCam2World = {};
calibPoseFileInd = 1;
while true
    try
        extCam2World{length(extCam2World)+1} = dlmread(calibPoseFile,'\t',[calibPoseFileInd,0,calibPoseFileInd+3,3]);
        calibPoseFileInd = calibPoseFileInd+6;
    catch
        break;
    end
end
