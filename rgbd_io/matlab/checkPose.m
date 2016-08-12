


seqPath = '/home/andyzeng/apc/toolbox/data/benchmark/warehouse/competition/stow/seq-0008';
calibDir = '/home/andyzeng/apc/toolbox/data/benchmark/warehouse/calibration';


close all;

seqData = loadSeq(seqPath);

% Apply calibrated camera poses
if strcmp(seqData.env,'tote')
    calibPath = fullfile(calibDir,'tote');
    calibPoseFilename = 'cam.poses.txt';
    seqData.extCam2World = loadCalib(fullfile(calibPath,calibPoseFilename));
end


[~,seqPC] = genPC(seqData);

% seqPts = seqPC.Location';

extWorld2Web = eye(4);
extWorld2Web(1,4) = -mean(seqPC.XLimits);
extWorld2Web(2,4) = -mean(seqPC.YLimits);
extWorld2Web(3,4) = -min(seqPC.ZLimits);

axesSwap = eye(4);
axesSwap(1:3,1:3) = vrrotvec2mat([1,0,0,-pi/2]);

extWorld2Web = axesSwap*extWorld2Web;

% seqPC = pctransform(seqPC,affine3d(extWorld2Web'));


objPC = pcread('/home/andyzeng/apc/rgbd-annotator/data/pointclouds/objects/soft_white_lightbulb.ply');

% tform = [-0.99496675,0.09196100,-0.03980378,0.00000000,-0.04522210,-0.05760696,0.99731457,0.00000000,0.08942107,0.99409485,0.06147571,0.00000000,-0.08077065,0.06262871,-0.04734460,1.00000000];
% wcs0008::
tform = [0.85999322,0.00000000,0.51030540,0.00000000,0.00000000,1.00000000,0.00000000,0.00000000,-0.51030540,0.00000000,0.85999322,0.00000000,-0.09992289,0.07086802,-0.06940980,1.00000000];
% tform = eye(4);
tform = reshape(tform,[4,4]);
tform = inv(extWorld2Web)*tform;%*extWorld2Web;


objPts = objPC.Location';
objPts = tform(1:3,1:3)*objPts + repmat(tform(1:3,4),1,size(objPts,2));

objPC = pointCloud(objPts','Color',repmat(uint8([0,255,0]),size(objPC.Color,1),1));


pcshow(seqPC); xlabel('x'); ylabel('y'); zlabel('z');
hold on; pcshow(objPC); hold off;


