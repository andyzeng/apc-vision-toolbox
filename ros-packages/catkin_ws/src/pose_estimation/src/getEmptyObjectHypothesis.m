function objectHypothesis = getEmptyObjectHypothesis(scenePath,objName,instanceIdx)

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
fid = fopen(fullfile(scenePath,strcat(objName,sprintf('.%d.result.txt',instanceIdx))),'w');
fprintf(fid,'%f\n',zeros(27,1));
fclose(fid);

end

