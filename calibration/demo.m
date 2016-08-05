% DEMO: Runs a demo script that creates a denoised point cloud from an 
% RGB-D sequence using calibrated camera poses.  Each RGB-D frame is 
% projected into 3D space as a point cloud, then aggregated with the point 
% clouds of the other frames. Note that calibration may not always give
% a better set of camera poses (extrinsics). This demo script saves a 
% point cloud into a .ply file, which can be visualized with a 3D viewer
% like Meshlab:
%   rgbd.ply   - 3D point cloud of all RGB-D frames projected into world
%               space (with calibrated camera poses and denoising)
%
% Author: Andy Zeng, andyz@princeton.edu

% Parameters
seqDir = '../sample_data/test/seq-000000';
gridFilterKernelSize = 0.001;

% Add toolbox paths
addpath(genpath('../rgbd_io/matlab'));

% Load RGB-D sequence
seqData = loadSeq(seqDir);

% Apply calibrated camera poses (if not found, perform calibration)
if strcmp(seqData.env,'shelf')
    calibDir = '../sample_data/calibration/shelf';
    calibPoseFilename = sprintf('cam.poses.%s.txt',seqData.binId);
else
    calibDir = '../sample_data/calibration/tote';
    calibPoseFilename = 'cam.poses.txt';
end
if ~exist(fullfile(calibDir,calibPoseFilename),'file')
    fprintf('Calibrated camera poses not found!\n');
    calibrate;
end
seqData.extCam2World = loadCalib(fullfile(calibDir,calibPoseFilename));

% Create aggregated RGB-D point cloud
fprintf('Creating 3D point cloud for "%s" ...',seqDir);
numFrames = length(seqData.colorFrames);
for frameIdx = 1:numFrames
    color = seqData.colorFrames{frameIdx};
    depth = seqData.depthFrames{frameIdx};
    extCam2World = seqData.extCam2World{frameIdx};

    % Project depth into camera space
    [pixX,pixY] = meshgrid(1:640,1:480);
    camX = (pixX-seqData.colorK(1,3)).*depth/seqData.colorK(1,1);
    camY = (pixY-seqData.colorK(2,3)).*depth/seqData.colorK(2,2);
    camZ = depth;
    
    % Only use points with valid depth
    validDepth = find((camZ > 0));
    camPts = [camX(validDepth),camY(validDepth),camZ(validDepth)]';
    camPts = extCam2World(1:3,1:3) * camPts + repmat(extCam2World(1:3,4),1,size(camPts,2));
    
    % Get vertex colors
    colorR = color(:,:,1);
    colorG = color(:,:,2);
    colorB = color(:,:,3);
    colorPts = [colorR(validDepth),colorG(validDepth),colorB(validDepth)]';
    
    % Aggregate point clouds
    if frameIdx == 1
      camCloud = pointCloud(camPts','Color',colorPts');
    else
      camCloud = pcmerge(camCloud,pointCloud(camPts','Color',colorPts'),gridFilterKernelSize);
    end
end

% Denoise point cloud
camCloud = pcdownsample(camCloud,'gridAverage',gridFilterKernelSize);
camCloud = pcdenoise(camCloud,'NumNeighbors',4*numFrames);
pcwrite(camCloud,'rgbd','PLYFormat','binary');
fprintf(' done!\n');
