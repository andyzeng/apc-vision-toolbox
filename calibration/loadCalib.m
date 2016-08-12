function seqData = loadCalib(calibDir,seqData)
% LOADCALIB Load the relative camera-to-camera poses generated from
% calibSeq and use them to fix the camera poses for the captured RGB-D 
% sequence saved in seqData.
%
% Arguments: (input)
%   calibPoseFile - file path to the file that contains the set of
%                   relative camera-to-camera poses generated from calibSeq
%   seqData       - data structure holding the contents (frames and camera
%                   information) of a captured RGB-D sequence.
%
% Arguments: (output)
%   extCam2World  - 1xn cell array of 4x4 camera-to-world extrinsic
%                   matrices (camera pose, in homogeneous coordinates)
%
% Author: Andy Zeng, andyz@princeton.edu

% Retrieve pivot viewpoint camera pose based on the sequence environment
if strcmp(seqData.env,'tote')
    baseFrameIdx = 1;
    calibPoseFile = fullfile(calibDir,'tote','cam.poses.txt');
else
    baseFrameIdx = 8;
    calibPoseFile = fullfile(calibDir,'shelf',sprintf('cam.poses.%s.txt',seqData.binId));
end
extPivot2World = seqData.extCam2World{baseFrameIdx};

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
for frameIdx = 1:length(seqData.colorFrames)
    seqData.extCam2World{frameIdx} = extPivot2World * extCam2Pivot{frameIdx};
end
