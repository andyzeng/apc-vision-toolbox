function objectHypothesis = getEmptyObjectHypothesis(scenePath,objName,instanceIdx)
% Return ROS message with empty object pose
%
% ---------------------------------------------------------
% Copyright (c) 2016, Andy Zeng
% 
% This file is part of the APC Vision Toolbox and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------

% Save object pose to ROS message
poseTrans = rosmessage('geometry_msgs/Point');
poseTrans.X = 0;
poseTrans.Y = 0;
poseTrans.Z = 0;
poseRot = rosmessage('geometry_msgs/Quaternion');
poseQuat = rotm2quat(eye(3));
poseRot.X = poseQuat(2);
poseRot.Y = poseQuat(3);
poseRot.Z = poseQuat(4);
poseRot.W = poseQuat(1);
poseMsg = rosmessage('geometry_msgs/Pose');
poseMsg.Position = poseTrans;
poseMsg.Orientation = poseRot;

% Save PCA to ROS message
pcaTrans = rosmessage('geometry_msgs/Point');
pcaTrans.X = 0;
pcaTrans.Y = 0;
pcaTrans.Z = 0;
pcaRot = rosmessage('geometry_msgs/Quaternion');
pcaQuat = rotm2quat(eye(3));
pcaRot.X = pcaQuat(2);
pcaRot.Y = pcaQuat(3);
pcaRot.Z = pcaQuat(4);
pcaRot.W = pcaQuat(1);
pcaMsg = rosmessage('geometry_msgs/Pose');
pcaMsg.Position = pcaTrans;
pcaMsg.Orientation = pcaRot;

% Save PCA variances to ROS message
pcaLatent = rosmessage('geometry_msgs/Point');
pcaLatent.X = 0;
pcaLatent.Y = 0;
pcaLatent.Z = 0;

% Save surface mean to ROS message
surfaceMean = rosmessage('geometry_msgs/Point');
surfaceMean.X = 0;
surfaceMean.Y = 0;
surfaceMean.Z = 0;

% Prepare ROS message 
objectHypothesis = rosmessage('pose_estimation/ObjectHypothesis');
objectHypothesis.Label = objName;
objectHypothesis.Pose = poseMsg;
objectHypothesis.Pca = pcaMsg;
objectHypothesis.Latent = pcaLatent;
objectHypothesis.Mean = surfaceMean;
objectHypothesis.RangeX = [0 0];
objectHypothesis.RangeY = [0 0];
objectHypothesis.RangeZ = [0 0];
objectHypothesis.Score = 0;

% Save ROS message to file
if ~exist(fullfile(scenePath,'results'),'file')
    mkdir(fullfile(scenePath,'results'));
end
fid = fopen(fullfile(scenePath,'results',strcat(objName,sprintf('.%d.result.txt',instanceIdx))),'w');
fprintf(fid,'# Predicted object pose translation (x,y,z in world coordinates)\n%15.8e\t %15.8e\t %15.8e\t\n\n',0,0,0);
fprintf(fid,'# Predicted object pose rotation (x,y,z,w object-to-world quaternion)\n%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n\n',0,0,0,0);
fprintf(fid,'# Segmented object point cloud median (x,y,z in world coordinates)\n%15.8e\t %15.8e\t %15.8e\t\n\n',0,0,0);
fprintf(fid,'# Segmented object point cloud PCA rotation (x,y,z,w object-to-world quaternion)\n%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n\n',0,0,0,0);
fprintf(fid,'# Segmented object point cloud PCA variance (in PC directions)\n%15.8e\t %15.8e\t %15.8e\t\n\n',0,0,0);
fprintf(fid,'# Segmented object point cloud centroid/mean (x,y,z in world coordinates)\n%15.8e\t %15.8e\t %15.8e\t\n\n',0,0,0);
fprintf(fid,'# Segmented object point cloud x-range in world coordinates (bounding box in x-direction)\n%15.8e\t %15.8e\t\n\n',0,0);
fprintf(fid,'# Segmented object point cloud y-range in world coordinates (bounding box in y-direction)\n%15.8e\t %15.8e\t\n\n',0,0);
fprintf(fid,'# Segmented object point cloud z-range in world coordinates (bounding box in z-direction)\n%15.8e\t %15.8e\t\n\n',0,0);
fprintf(fid,'# Prediction confidence score\n%.17g\n',0);
fclose(fid);

end

