% DEMO: Runs a demo script that creates a point cloud from an RGB-D 
% sequence.  Each RGB-D frame is projected into 3D space as a point cloud, 
% then aggregated with the point clouds of the other frames. This demo 
% script saves a point cloud into a .ply file, which can be visualized 
% with a 3D viewer like Meshlab:
%   rgbd.ply   - 3D point cloud of all RGB-D frames projected into world
%               space (with denoising)
%
% Author: Andy Zeng, andyz@princeton.edu

% Parameters
seqDir = '../../sample_data/test/seq-000000';

% Load RGB-D sequence
seqData = loadSeq(seqDir);

% Create aggregated RGB-D point cloud
fprintf('Creating 3D point cloud for "%s" ...',seqDir);
numFrames = length(seqData.colorFrames);
allCamPoints = [];
allCamColors = [];
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
    allCamPoints = [allCamPoints,camPts];
    allCamColors = [allCamColors,colorPts];
end

% Save aggregated RGB-D point cloud
pcwrite(pointCloud(allCamPoints','Color',allCamColors'),'rgbd','PLYFormat','binary');
fprintf(' done!\n');