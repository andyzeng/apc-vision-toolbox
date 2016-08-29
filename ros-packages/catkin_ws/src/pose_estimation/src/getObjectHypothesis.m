function objectHypothesis = getObjectHypothesis(RtPCA,latentPCA,surfaceCentroid,surfaceRange,finalRt,finalScore,dataPath,objName,instanceIdx)

% Save object pose to ROS message
poseTrans = rosmessage('geometry_msgs/Point');
poseTrans.X = finalRt(1,4);
poseTrans.Y = finalRt(2,4);
poseTrans.Z = finalRt(3,4);
poseRot = rosmessage('geometry_msgs/Quaternion');
poseQuat = rotm2quat(finalRt(1:3,1:3));
poseRot.X = poseQuat(2);
poseRot.Y = poseQuat(3);
poseRot.Z = poseQuat(4);
poseRot.W = poseQuat(1);
poseMsg = rosmessage('geometry_msgs/Pose');
poseMsg.Position = poseTrans;
poseMsg.Orientation = poseRot;

% Save PCA to ROS message
pcaTrans = rosmessage('geometry_msgs/Point');
pcaTrans.X = RtPCA(1,4);
pcaTrans.Y = RtPCA(2,4);
pcaTrans.Z = RtPCA(3,4);
pcaRot = rosmessage('geometry_msgs/Quaternion');
pcaQuat = rotm2quat(RtPCA(1:3,1:3));
pcaRot.X = pcaQuat(2);
pcaRot.Y = pcaQuat(3);
pcaRot.Z = pcaQuat(4);
pcaRot.W = pcaQuat(1);
pcaMsg = rosmessage('geometry_msgs/Pose');
pcaMsg.Position = pcaTrans;
pcaMsg.Orientation = pcaRot;

% Save PCA variances to ROS message
pcaLatent = rosmessage('geometry_msgs/Point');
pcaLatent.X = latentPCA(1)*50;
pcaLatent.Y = latentPCA(2)*50;
pcaLatent.Z = latentPCA(3)*50;

% Save surface mean to ROS message
surfaceMean = rosmessage('geometry_msgs/Point');
surfaceMean.X = surfaceCentroid(1);
surfaceMean.Y = surfaceCentroid(2);
surfaceMean.Z = surfaceCentroid(3);

% Prepare ROS message 
objectHypothesis = rosmessage('pose_estimation/ObjectHypothesis');
objectHypothesis.Label = objName;
objectHypothesis.Pose = poseMsg;
objectHypothesis.Pca = pcaMsg;
objectHypothesis.Latent = pcaLatent;
objectHypothesis.Mean = surfaceMean;
objectHypothesis.RangeX = surfaceRange(1,:);
objectHypothesis.RangeY = surfaceRange(2,:);
objectHypothesis.RangeZ = surfaceRange(3,:);
objectHypothesis.Score = finalScore;

% Save ROS message to file
fid = fopen(fullfile(dataPath,strcat(objName,sprintf('.%d.result.txt',instanceIdx))),'w');
fprintf(fid,'%f\n%f\n%f\n',finalRt(1,4),finalRt(2,4),finalRt(3,4));
fprintf(fid,'%f\n%f\n%f\n%f\n',poseQuat(2),poseQuat(3),poseQuat(4),poseQuat(1));
fprintf(fid,'%f\n%f\n%f\n',RtPCA(1,4),RtPCA(2,4),RtPCA(3,4));
fprintf(fid,'%f\n%f\n%f\n%f\n',pcaQuat(2),pcaQuat(3),pcaQuat(4),pcaQuat(1));
fprintf(fid,'%f\n%f\n%f\n',latentPCA(1)*50,latentPCA(2)*50,latentPCA(3)*50);
fprintf(fid,'%f\n%f\n%f\n',surfaceCentroid(1),surfaceCentroid(2),surfaceCentroid(3));
fprintf(fid,'%f\n%f\n',surfaceRange(1,1),surfaceRange(1,2));
fprintf(fid,'%f\n%f\n',surfaceRange(2,1),surfaceRange(2,2));
fprintf(fid,'%f\n%f\n',surfaceRange(3,1),surfaceRange(3,2));
fprintf(fid,'%f\n',finalScore);
fclose(fid);
  
end

