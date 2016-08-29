clear all; close all;

% alignPath = 'C:/Users/andyz/Documents/apcdata/tote_align/000000';
% savetoPath = 'C:/Users/andyz/Documents/apcdata/tote_align/poses';
% alignPath = '/home/mcube/apcdata/princeton_data/raw/tote_align/000000';
% savetoPath = '/home/mcube/apcdata/princeton_data/raw/tote_align/poses';
% alignPath = '/home/mcube/apcdata/princeton_data/raw/tote_test6/calib';
% savetoPath = '/home/mcube/apcdata/princeton_data/raw/tote_test6/poses';
alignPath = '/home/mcube/apcdata/princeton_data/raw/tote_align/calib';
savetoPath = '/home/mcube/apcdata/princeton_data/raw/tote_align/poses';


% run('lib/vlfeat/toolbox/vl_setup.m');
% alignDir = dir(alignPath);
addpath(fullfile('lib/peter'));
addpath(fullfile('lib/estimateRigidTransform'));
addpath(fullfile('lib/sfm'));
mkdir(savetoPath);

% Read RGB-D frames
allCamPts = [];
allColorPts = [];
depthFiles = dir(fullfile(alignPath,'*.regdepth.png'));
colorFiles = dir(fullfile(alignPath,'*.color.png'));
extCam2WorldFiles = dir(fullfile(alignPath,'*.pose_camera_map.txt'));
colorK = dlmread(fullfile(alignPath,'color_intrinsics.K.txt'));
extWorld2Bin = inv(dlmread(fullfile(alignPath,'pose_bin-1_map.txt')));

allColorIm = {};
allDepthIm = {};
allExtCam2World = {};
allCamPts = {};
allColorPts = {};
allSURF2D = {};
allSURF3D = {};
allSURFDesc = {};
% newExt = {};
for frameIdx = 1:length(extCam2WorldFiles)
    colorIm = imread(fullfile(alignPath,colorFiles(frameIdx).name));
    depthIm = double(imread(fullfile(alignPath,depthFiles(frameIdx).name)))./10000;
    extCam2World = dlmread(fullfile(alignPath,extCam2WorldFiles(frameIdx).name));
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
    if frameIdx == 1
      allExtCam2World{frameIdx} = extCam2World;
      dlmwrite(fullfile(savetoPath,extCam2WorldFiles(frameIdx).name),allExtCam2World{frameIdx},'delimiter',' ');
      continue;
    elseif mod(frameIdx,3) == 1
      prevFrameIdx = frameIdx - 3;
    else
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
%     currCamPts = currExtCam2World(1:3,1:3) * allCamPts{frameIdx} + repmat(currExtCam2World(1:3,4),1,size(allCamPts{frameIdx},2));
%     currCloud = pointCloud(currCamPts','Color',allColorPts{frameIdx}');
%     pcwrite(currCloud,sprintf('%d',frameIdx),'PLYFormat','binary');
    dlmwrite(fullfile(savetoPath,extCam2WorldFiles(frameIdx).name),currExtCam2World,'delimiter',' ');

end














