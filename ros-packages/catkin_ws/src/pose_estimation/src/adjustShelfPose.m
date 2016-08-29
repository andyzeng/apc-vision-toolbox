clear all; close all;

alignPath = '/home/mcube/apcdata/princeton_data/raw/shelf_align';
savetoPath = '/home/mcube/apcdata/princeton_data/raw/shelf_align/poses';

% Align all frames to the middle frame (000007)

% run('lib/vlfeat/toolbox/vl_setup.m');
% alignDir = dir(alignPath);
addpath(fullfile('lib/peter'));
addpath(fullfile('lib/estimateRigidTransform'));
addpath(fullfile('lib/sfm'));

alignDir = dir(fullfile(alignPath,'00*'));

for seqIdx = 1:length(alignDir)

  seqPath = fullfile(alignPath,alignDir(seqIdx).name);
  seqSaveToPath = fullfile(savetoPath,alignDir(seqIdx).name);
  mkdir(seqSaveToPath);
  
  % Read RGB-D frames
  allCamPts = [];
  allColorPts = [];
  depthFiles = dir(fullfile(seqPath,'*.regdepth.png'));
  colorFiles = dir(fullfile(seqPath,'*.color.png'));
  extCam2WorldFiles = dir(fullfile(seqPath,'*.pose_camera_map.txt'));
  colorK = dlmread(fullfile(seqPath,'color_intrinsics.K.txt'));
  extWorld2Bin = inv(dlmread(fullfile(seqPath,sprintf('pose_bin%d_map.txt',seqIdx-1))));

  allColorIm = {};
  allDepthIm = {};
  allExtCam2World = {};
  allCamPts = {};
  allColorPts = {};
  allSURF2D = {};
  allSURF3D = {};
  allSURFDesc = {};
  % newExt = {};
  for frameIdx = [8 7 6 9 10 3 2 1 4 5 13 12 11 14 15] %1:length(extCam2WorldFiles)
      colorIm = imread(fullfile(seqPath,colorFiles(frameIdx).name));
      depthIm = double(imread(fullfile(seqPath,depthFiles(frameIdx).name)))./10000;
%       extCam2World = dlmread(fullfile(seqPath,extCam2WorldFiles(frameIdx).name));
      allColorIm{frameIdx} = colorIm;
      allDepthIm{frameIdx} = depthIm;

      % Project depth into camera space
      [pixX,pixY] = meshgrid(1:640,1:480);
      camX = (pixX-colorK(1,3)).*allDepthIm{frameIdx}/colorK(1,1);
      camY = (pixY-colorK(2,3)).*allDepthIm{frameIdx}/colorK(2,2);
      camZ = allDepthIm{frameIdx};

      % Only use points with valid depth
      validDepth = find(camZ > 0);
      camPts = [camX(validDepth),camY(validDepth),camZ(validDepth)]';
      allCamPts{frameIdx} = camPts;
  %     allCamPts{frameIdx} = extCam2World(1:3,1:3) * camPts + repmat(extCam2World(1:3,4),1,size(camPts,2));

      % Get vertex colors
      colorR = colorIm(:,:,1);
      colorG = colorIm(:,:,2);
      colorB = colorIm(:,:,3);
      allColorPts{frameIdx} = [colorR(validDepth),colorG(validDepth),colorB(validDepth)]';

      % Extract SURF features of current frame
      currFrameSURF = detectSURFFeatures(rgb2gray(colorIm));
      [currFrameDesc,currFrameSURF] = extractFeatures(rgb2gray(colorIm), currFrameSURF);

      % Get SURF points in 3D of current frame
      currSURFpts3D = [];
      depthIm = allDepthIm{frameIdx};
      for SURFIdx = 1:length(currFrameSURF)
          currSURF2D = round(currFrameSURF(SURFIdx).Location');
          currSURF3D = zeros(3,1);
          currSURF3D(1) = (currSURF2D(1)-colorK(1,3)).*depthIm(currSURF2D(2),currSURF2D(1))/colorK(1,1);
          currSURF3D(2) = (currSURF2D(2)-colorK(2,3)).*depthIm(currSURF2D(2),currSURF2D(1))/colorK(2,2);
          currSURF3D(3) = depthIm(currSURF2D(2),currSURF2D(1));
  %         currSURF3D = extCam2World(1:3,1:3) * currSURF3D + repmat(extCam2World(1:3,4),1,size(currSURF3D,2));
          currSURFpts3D = [currSURFpts3D currSURF3D]; 
      end

      % Only use SURF points with valid depth
      validDepth = find(currSURFpts3D(3,:) > 0.15);
      currFrameSURF = currFrameSURF(validDepth);
      currFrameDesc = currFrameDesc(validDepth,:);
      currSURFpts3D = currSURFpts3D(:,validDepth);
      allSURF2D{frameIdx} = currFrameSURF;
      allSURFDesc{frameIdx} = currFrameDesc;
      allSURF3D{frameIdx} = currSURFpts3D;

      % Get index of previous frame
      if frameIdx == 8
%         allExtCam2World{frameIdx} = extCam2World;
        allExtCam2World{frameIdx} = eye(4);
        dlmwrite(fullfile(seqSaveToPath,extCam2WorldFiles(frameIdx).name),allExtCam2World{frameIdx},'delimiter',' ');
        continue;
      elseif mod(frameIdx,5) == 3
        prevFrameIdx = 8;
      elseif mod(frameIdx,5) == 2
        prevFrameIdx = frameIdx + 1;
      elseif mod(frameIdx,5) == 1
        prevFrameIdx = frameIdx + 1;
      elseif mod(frameIdx,5) == 4
        prevFrameIdx = frameIdx - 1;
      elseif mod(frameIdx,5) == 0
        prevFrameIdx = frameIdx - 1;
      end

      % Get SURF features of previous frame
      prevFrameSURF = allSURF2D{prevFrameIdx};
      prevFrameDesc = allSURFDesc{prevFrameIdx};
      prevSURFpts3D = allSURF3D{prevFrameIdx};

      % Match SURF features
      matchInd = matchFeatures(prevFrameDesc,currFrameDesc);
      prevMatchSURF  = prevFrameSURF(matchInd(:,1));
      currMatchSURF = currFrameSURF(matchInd(:,2));
      prevMatchPts3D  = prevSURFpts3D(:,matchInd(:,1));
      currMatchPts3D = currSURFpts3D(:,matchInd(:,2));
  %     showMatchedFeatures(allColorIm{prevFrameIdx},allColorIm{frameIdx},prevMatchSURF,currMatchSURF);

      [initRt, ransacInliers] = ransacfitRt([prevMatchPts3D;currMatchPts3D], 0.005,0);
      fprintf('RANSAC inliers: %d/%d\n',size(ransacInliers,2),size(currMatchPts3D,2));

      allExtCam2World{frameIdx} = allExtCam2World{prevFrameIdx} * [initRt; [0 0 0 1]];
      currExtCam2World = allExtCam2World{frameIdx};
      
%       currCamPts = currExtCam2World(1:3,1:3) * allCamPts{frameIdx} + repmat(currExtCam2World(1:3,4),1,size(allCamPts{frameIdx},2));
%       currCloud = pointCloud(currCamPts','Color',allColorPts{frameIdx}');
%       pcwrite(currCloud,sprintf('%d',frameIdx),'PLYFormat','binary');
  
      dlmwrite(fullfile(seqSaveToPath,extCam2WorldFiles(frameIdx).name),currExtCam2World,'delimiter',' ');

  end

end












