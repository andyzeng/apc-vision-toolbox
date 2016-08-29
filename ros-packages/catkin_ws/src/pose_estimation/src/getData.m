function data = getData(dataPath,camPosePath,frames,binID)

% Search through files in data directory
colorFiles = dir(fullfile(dataPath,'*.color.png'));
depthFiles = dir(fullfile(dataPath,'*.regdepth.png'));
if binID == -1
  extFiles = dir(fullfile(camPosePath,'*.pose_camera_map.txt'));
else
  extFiles = dir(fullfile(dataPath,'*.pose_camera_map.txt'));
end

% Load camera intrinsics
data.K = dlmread(fullfile(dataPath,'color_intrinsics.K.txt'));

% Load RGB-D images and camera poses
data.extBin2World = dlmread(fullfile(dataPath,sprintf('pose_bin%d_map.txt',binID)));
data.extWorld2Bin = inv(data.extBin2World);
data.color = {};
data.depth = {};
data.extCam2World = {};
for frameIdx = frames
    data.color{frameIdx} = imread(fullfile(dataPath,colorFiles(frameIdx).name));
    data.depth{frameIdx} = double(imread(fullfile(dataPath,depthFiles(frameIdx).name)))./10000;
    if binID == -1
      data.extCam2World{frameIdx} = dlmread(fullfile(camPosePath,extFiles(frameIdx).name));
    else
      midExtCam2World = dlmread(fullfile(dataPath,extFiles(8).name));
      data.extCam2World{frameIdx} = midExtCam2World * dlmread(fullfile(camPosePath,sprintf('%06d',binID),extFiles(frameIdx).name));
    end
end

end

