
binID = 11;
gridSampleSize = 0.002;

% datapath = sprintf('/home/mcube/apcdata/princeton_data/raw/tote_test3/empty');
% totePosePath = '/home/mcube/apcdata/princeton_data/raw/tote_align/poses';
% datapath = sprintf('/home/mcube/apcdata/princeton_data/raw/tote_align/empty');
% totePosePath = '/home/mcube/apcdata/princeton_data/raw/tote_align/poses';
datapath = sprintf('/home/mcube/apcdata/princeton_data/raw/shelf_align/empty/000011');
totePosePath = '/home/mcube/apcdata/princeton_data/raw/shelf_align/poses/000011';

if binID ~= -1
  posePath = datapath;
else
  posePath = totePosePath;
end


% Read RGB-D frames
fprintf('Reading RGB-D frames.\n');
depthFiles = dir(fullfile(datapath,'*.regdepth.png'));
colorFiles = dir(fullfile(datapath,'*.color.png'));
extFiles = dir(fullfile(posePath,'*.pose_camera_map.txt'));
extWorld2Bin = inv(dlmread(fullfile(datapath,sprintf('pose_bin%d_map.txt',binID))));
depthArray = {};
colorImArray = {};
extCam2WorldArray = {};
for frameIdx = 1:length(depthFiles)
    depthArray{frameIdx} = double(imread(fullfile(datapath,depthFiles(frameIdx).name)))./10000;
    colorImArray{frameIdx} = imread(fullfile(datapath,colorFiles(frameIdx).name));
    extCam2WorldArray{frameIdx} = dlmread(fullfile(posePath,extFiles(frameIdx).name));
end

% Get camera info
colorK = dlmread(fullfile(datapath,'color_intrinsics.K.txt'));
% depthK = dlmread(fullfile(datapath,'depth_intrinsics.K.txt'));
% depth2colorExt = dlmread(fullfile(datapath,'depth2color_extrinsics.K.txt'));

% Create aggregated RGB-D point cloud (15 frames)
fprintf('Aggregating RGB-D point cloud.\n');
allCamPts = [];
allColorPts = [];
objCamPts = [];
for frameIdx = 1:length(depthFiles)
    depth = depthArray{frameIdx};
    colorIm = colorImArray{frameIdx};
    extCam2World = extCam2WorldArray{frameIdx};
    extCam2Bin = extWorld2Bin * extCam2World;

    % Project depth into camera space
    [pixX,pixY] = meshgrid(1:640,1:480);
    camX = (pixX-colorK(1,3)).*depth/colorK(1,1);
    camY = (pixY-colorK(2,3)).*depth/colorK(2,2);
    camZ = depth;
    
    % Only use points with valid depth
    validDepth = intersect(find(camZ > 0),find(camZ < 1));
    camPts = [camX(validDepth),camY(validDepth),camZ(validDepth)]';
    camPts = extCam2Bin(1:3,1:3) * camPts + repmat(extCam2Bin(1:3,4),1,size(camPts,2));
    allCamPts = [allCamPts camPts];
    
    % Get vertex colors
    colorR = colorIm(:,:,1);
    colorG = colorIm(:,:,2);
    colorB = colorIm(:,:,3);
    colorPts = [colorR(validDepth),colorG(validDepth),colorB(validDepth)]';
    allColorPts = [allColorPts colorPts];
    
%     pcwrite(pointCloud(camPts','Color',colorPts'),fullfile('bins',sprintf('test%d',frameIdx)),'PLYFormat','binary');
end

ptCloud = pointCloud(allCamPts','Color',allColorPts');
ptCloud = pcdownsample(ptCloud,'gridAverage',0.005);
pcwrite(ptCloud,fullfile('bins',sprintf('bin%d',binID)),'PLYFormat','binary');






















