close all;


seqPath = '/home/andyzeng/apc/toolbox/data/benchmark/office/test/shelf/seq-0006';
calibPath = '/home/andyzeng/apc/toolbox/data/benchmark/warehouse/calibration';
objName = 'cool_shot_glue_sticks';
label = [0.99906045,0.04333844,-0.00000001,0.00000000,-0.00161355,0.03719663,0.99930668,0.00000000,0.04330839,-0.99836779,0.03723161,0.00000000,-0.07803328,0.05557691,0.01329018,1.00000000];

% Load RGB-D sequence data
seqData = loadSeq(seqPath);

% Apply calibrated camera poses
addpath(genpath('../../calibration'));
seqData = loadCalib(calibPath,seqData);

% Create point cloud from sequence data
[~,seqPC] = genPC(seqData);

% Compute coordinate frame transform from world space to web space
extWorld2Web = eye(4);
extWorld2Web(1,4) = -mean(seqPC.XLimits);
extWorld2Web(2,4) = -mean(seqPC.YLimits);
extWorld2Web(3,4) = -min(seqPC.ZLimits);
axesSwap = eye(4);
axesSwap(1:3,1:3) = vrrotvec2mat([1,0,0,-pi/2]);
extWorld2Web = axesSwap*extWorld2Web;

% Apply labeled pose
objPC = pcread(sprintf('/home/andyzeng/apc/rgbd-annotator/data/pointclouds/objects/%s.ply',objName));
tform = reshape(label,[4,4]);
tform = inv(extWorld2Web)*tform;%*extWorld2Web;
objPts = objPC.Location';
objPts = tform(1:3,1:3)*objPts + repmat(tform(1:3,4),1,size(objPts,2));
objPC = pointCloud(objPts','Color',repmat(uint8([0,255,0]),size(objPC.Color,1),1));

% Display object pose label
pcshow(seqPC); xlabel('x'); ylabel('y'); zlabel('z');
hold on; pcshow(objPC); hold off;


