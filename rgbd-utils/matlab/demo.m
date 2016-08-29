% Runs a demo script that creates a point cloud from an RGB-D sequence.  
% 
% The camera poses of the RGB-D sequence are first calibrated. Then each 
% RGB-D frame is hole-filled, projected into 3D space as a point cloud,
% aggregated with the point clouds of the other frames, and denoised using
% a grid filter. This demo script saves a point cloud into a .ply file, 
% which can be visualized with a 3D viewer like Meshlab:
%   rgbd.ply   - 3D point cloud of all RGB-D frames projected into world
%               space (with denoising)
%
% ---------------------------------------------------------
% Copyright (c) 2016, Andy Zeng
% 
% This file is part of the APC Vision Toolbox and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------

% User configurations (change me)
scenePath = '../../data/benchmark/warehouse/competition/shelf/scene-0006';
doCalib = true;
calibDir = '../../data/benchmark/warehouse/calibration';

% Load RGB-D sequence
sceneData = loadScene(scenePath);

% Apply calibrated camera poses
if doCalib
    if strcmp(sceneData.env,'shelf')
        calibPoseFilename = sprintf('cam.poses.%s.txt',sceneData.binId);
    else
        calibPoseFilename = 'cam.poses.txt';
    end

    % If calibrated camera pose files are not found, perform calibration
    if ~exist(fullfile(calibDir,'tote','cam.poses.txt'),'file') || ~exist(fullfile(calibDir,'shelf','cam.poses.L.txt'),'file')
        fprintf('Calibrated camera poses not found!\n');

        % Calibrate camera poses for tote views
        fprintf('Calibrating tote views...');
        calibDirTote = fullfile(calibDir,'tote');
        getCalib(fullfile(calibDirTote,'scene-tote'),fullfile(calibDirTote,'cam.poses.txt'));
        fprintf(' done!\n');

        % Calibrate camera poses for all shelf bin views
        fprintf('Calibrating shelf views...');
        calibDirShelf = fullfile(calibDir,'shelf');
        binIds = 'ABCDEFGHIJKL';
        for binIdx = 1:length(binIds)
            fprintf(sprintf(' %s',binIds(binIdx)));
            getCalib(fullfile(calibDirShelf,strcat('scene-',binIds(binIdx))),fullfile(calibDirShelf,sprintf('cam.poses.%s.txt',binIds(binIdx))));
        end
        fprintf(' done!\n');
    end
    sceneData = loadCalib(calibDir,sceneData);
end

% Create aggregated RGB-D point cloud
fprintf('Creating 3D point cloud for "%s" ...',scenePath);
numFrames = length(sceneData.colorFrames);
allCamPoints = [];
allCamColors = [];
for frameIdx = 1:numFrames
    color = sceneData.colorFrames{frameIdx};
    depth = sceneData.depthFrames{frameIdx};
    extCam2World = sceneData.extCam2World{frameIdx};

    % Project depth into camera space
    [pixX,pixY] = meshgrid(1:640,1:480);
    camX = (pixX-sceneData.colorK(1,3)).*depth/sceneData.colorK(1,1);
    camY = (pixY-sceneData.colorK(2,3)).*depth/sceneData.colorK(2,2);
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
    allCamPoints = [allCamPoints,camPts];
    allCamColors = [allCamColors,colorPts];
end

% Denoise aggregated RGB-D point cloud
gridStep = 0.001;
camPointCloud = pointCloud(allCamPoints','Color',allCamColors');
camPointCloud = pcdownsample(camPointCloud,'gridAverage',gridStep);
camPointCloud = pcdenoise(camPointCloud,'NumNeighbors',4*numFrames);

% Save point cloud
pcwrite(camPointCloud,'rgbd','PLYFormat','binary');
fprintf(' done!\n');