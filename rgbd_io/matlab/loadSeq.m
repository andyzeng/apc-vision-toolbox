function data = loadSeq(path)
% LOADSEQ: Load camera information and a sequence of RGB-D frames.
%
% Arguments: (input)
%   path                - file path to the directory holding the RGB-D
%                         frame images (frame-XXXXXX.*.png) and camera 
%                         information file (cam.info.txt)
%
% Arguments: (output)
%   data.env            - environment in which the sequence was taken
%                         (can be 'shelf' or 'tote')
%   data.binId          - bin ID (can be 'A','B'...) in which sequence was 
%                         taken (only applicable if environment is 'shelf')
%   data.colorK         - 3x3 color camera intrinsic matrix
%   data.depthK         - 3x3 depth camera intrinsic matrix
%   data.extDepth2Color - 4x4 depth-to-color camera extrinsic matrix (in
%                         homogeneous coordinates, be used to align raw 
%                         depth images to color images)
%   data.extWorld2Bin   - 4x4 world-to-bin transformation matrix (in
%                         homogeneous coordinates)
%   (alternatively known as the camera pose matrix,
%   data.colorFrames    - 1xn cell array of 480x640x3 uint8 arrays of RGB
%                         color values (0 - 255)
%   data.depthFrames    - 1xn cell array of 480x640 float arrays of depth
%                         values in meters
%   data.extCam2World   - 1xn cell array of 4x4 camera-to-world extrinsic
%                         matrices (camera pose, in homogeneous coordinates)
%
% Author: Andy Zeng, andyz@princeton.edu

% Read camera info and object list
camInfoFile = fullfile(path,'cam.info.txt');
camInfoFileId = fopen(camInfoFile,'rb');
data.env = fscanf(camInfoFileId,'# Environment: %s');
if strcmp(data.env,'shelf')
    data.binId = fscanf(camInfoFileId,'\n# Bin ID: %s');
else
    fgetl(camInfoFileId);
    fgetl(camInfoFileId);
end
objListLine = fgetl(camInfoFileId);
objListDelim = strsplit(objListLine,'"');
data.objects = {};
for objIdx = 2:2:length(objListDelim)
    data.objects{length(data.objects)+1} = objListDelim{objIdx};
end
fclose(camInfoFileId);
data.colorK = dlmread(camInfoFile,'\t',[5,0,7,2]);
data.depthK = dlmread(camInfoFile,'\t',[10,0,12,2]);
data.extDepth2Color = dlmread(camInfoFile,'\t',[15,0,18,3]);
data.extBin2World = dlmread(camInfoFile,'\t',[21,0,24,3]);

% Read RGB-D frames and respective camera poses
colorFrameDir = dir(fullfile(path,'frame-*.color.png'));
depthFrameDir = dir(fullfile(path,'frame-*.depth.png'));
numFrames = min(length(colorFrameDir),length(depthFrameDir));
data.colorFrames = cell(1,numFrames);
data.depthFrames = cell(1,numFrames);
data.extCam2World = cell(1,numFrames);
for frameIdx = 1:numFrames
    data.colorFrames{frameIdx} = imread(fullfile(path,colorFrameDir(frameIdx).name));
    data.depthFrames{frameIdx} = readDepth(fullfile(path,depthFrameDir(frameIdx).name));
    data.extCam2World{frameIdx} = dlmread(camInfoFile,'\t',[21+6*frameIdx,0,24+6*frameIdx,3]);
end
