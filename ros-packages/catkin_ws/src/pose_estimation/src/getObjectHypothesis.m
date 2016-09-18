function objectHypothesis = getObjectHypothesis(surfPCAPoseWorld,latentPCA,surfCentroid,surfRangeWorld,predObjPoseWorld,predObjConfScore,scenePath,objName,instanceIdx)
% Return ROS message with predicted object pose information
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
try
    poseTrans = rosmessage('geometry_msgs/Point');
catch
end
poseTrans.X = predObjPoseWorld(1,4);
poseTrans.Y = predObjPoseWorld(2,4);
poseTrans.Z = predObjPoseWorld(3,4);
try
    poseRot = rosmessage('geometry_msgs/Quaternion');
catch
end
try
    poseQuat = rotm2quat(predObjPoseWorld(1:3,1:3));
catch
    poseQuat = rot2quat(predObjPoseWorld(1:3,1:3));
end
poseRot.X = poseQuat(2);
poseRot.Y = poseQuat(3);
poseRot.Z = poseQuat(4);
poseRot.W = poseQuat(1);
try
    poseMsg = rosmessage('geometry_msgs/Pose');
catch
end
poseMsg.Position = poseTrans;
poseMsg.Orientation = poseRot;

% Save PCA to ROS message
try
    pcaTrans = rosmessage('geometry_msgs/Point');
catch
end
pcaTrans.X = surfPCAPoseWorld(1,4);
pcaTrans.Y = surfPCAPoseWorld(2,4);
pcaTrans.Z = surfPCAPoseWorld(3,4);
try
    pcaRot = rosmessage('geometry_msgs/Quaternion');
catch
end
try
    pcaQuat = rotm2quat(surfPCAPoseWorld(1:3,1:3));
catch
    pcaQuat = rot2quat(surfPCAPoseWorld(1:3,1:3));
end
pcaRot.X = pcaQuat(2);
pcaRot.Y = pcaQuat(3);
pcaRot.Z = pcaQuat(4);
pcaRot.W = pcaQuat(1);
try
    pcaMsg = rosmessage('geometry_msgs/Pose');
catch
end
pcaMsg.Position = pcaTrans;
pcaMsg.Orientation = pcaRot;

% Save PCA variances to ROS message
try
    pcaLatent = rosmessage('geometry_msgs/Point');
catch
end
pcaLatent.X = latentPCA(1);
pcaLatent.Y = latentPCA(2);
pcaLatent.Z = latentPCA(3);

% Save surface mean to ROS message
try
    surfaceMean = rosmessage('geometry_msgs/Point');
catch
end
surfaceMean.X = surfCentroid(1);
surfaceMean.Y = surfCentroid(2);
surfaceMean.Z = surfCentroid(3);

% Prepare ROS message 
try
    objectHypothesis = rosmessage('pose_estimation/ObjectHypothesis');
catch
end
objectHypothesis.Label = objName;
objectHypothesis.Pose = poseMsg;
objectHypothesis.Pca = pcaMsg;
objectHypothesis.Latent = pcaLatent;
objectHypothesis.Mean = surfaceMean;
objectHypothesis.RangeX = surfRangeWorld(1,:);
objectHypothesis.RangeY = surfRangeWorld(2,:);
objectHypothesis.RangeZ = surfRangeWorld(3,:);
objectHypothesis.Score = predObjConfScore;

% Save ROS message to file
if ~exist(fullfile(scenePath,'results'),'file')
    mkdir(fullfile(scenePath,'results'));
end
fid = fopen(fullfile(scenePath,'results',strcat(objName,sprintf('.%d.result.txt',instanceIdx))),'w');
fprintf(fid,'# Predicted object pose translation (x,y,z in world coordinates)\n%15.8e\t %15.8e\t %15.8e\t\n\n',predObjPoseWorld(1,4),predObjPoseWorld(2,4),predObjPoseWorld(3,4));
fprintf(fid,'# Predicted object pose rotation (x,y,z,w object-to-world quaternion)\n%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n\n',poseQuat(2),poseQuat(3),poseQuat(4),poseQuat(1));
fprintf(fid,'# Segmented object point cloud median (x,y,z in world coordinates)\n%15.8e\t %15.8e\t %15.8e\t\n\n',surfPCAPoseWorld(1,4),surfPCAPoseWorld(2,4),surfPCAPoseWorld(3,4));
fprintf(fid,'# Segmented object point cloud PCA rotation (x,y,z,w object-to-world quaternion)\n%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n\n',pcaQuat(2),pcaQuat(3),pcaQuat(4),pcaQuat(1));
fprintf(fid,'# Segmented object point cloud PCA variance (in PC directions)\n%15.8e\t %15.8e\t %15.8e\t\n\n',latentPCA(1),latentPCA(2),latentPCA(3));
fprintf(fid,'# Segmented object point cloud centroid/mean (x,y,z in world coordinates)\n%15.8e\t %15.8e\t %15.8e\t\n\n',surfCentroid(1),surfCentroid(2),surfCentroid(3));
fprintf(fid,'# Segmented object point cloud x-range in world coordinates (bounding box in x-direction)\n%15.8e\t %15.8e\t\n\n',surfRangeWorld(1,1),surfRangeWorld(1,2));
fprintf(fid,'# Segmented object point cloud y-range in world coordinates (bounding box in y-direction)\n%15.8e\t %15.8e\t\n\n',surfRangeWorld(2,1),surfRangeWorld(2,2));
fprintf(fid,'# Segmented object point cloud z-range in world coordinates (bounding box in z-direction)\n%15.8e\t %15.8e\t\n\n',surfRangeWorld(3,1),surfRangeWorld(3,2));
fprintf(fid,'# Prediction confidence score\n%.17g\n',predObjConfScore);
fclose(fid);
  
end

