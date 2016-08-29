function HHA = getHHA(environment,depth,colorK,extCam2World,extBin2World)
% Convert depth image into HHA (horizontal disparity, height above ground,
% angle with gravity) feature map
%
% Based on:
%   Gupta, Saurabh, et al. "Learning rich features from RGB-D images for 
%     object detection and segmentation." European Conference on Computer 
%     Vision. Springer International Publishing, 2014.
%   Code reference: https://github.com/s-gupta/rgbdutils
%
% function HHA = getHHA(environment,depth,colorK,extCam2World,extBin2World)
% Input:
%   environment  - string 'shelf' or 'tote'
%   depth        - 480x640 float array of depth values in meters
%   colorK       - 3x3 color camera intrinsic matrix
%   extCam2World - 4x4 camera-to-world transformation matrix (in
%                  homogeneous coordinates)
%   extBin2World - 4x4 bin-to-world (or tote-to-world) transformation 
%                  matrix (in homogeneous coordinates)
% Output:
%   HHA          - 480x640x3 uint8 array of HHA feature values (0 - 255)
%
% ---------------------------------------------------------

addpath(genpath('external'));

% Define min/max depth ranges for disparity
if strcmp(environment,'shelf')
    depthFloor = 0.3;
    depthCeil = 0.8;
else
    depthFloor = 0.2;
    depthCeil = 0.6;
end

% Compute disparity
disparity = ones(size(depth(:)))'./depth(:)';
disparity = (disparity(:)-(1/depthCeil))./((1/depthFloor)-(1/depthCeil));
disparity(find(isinf(disparity))) = 0;
disparity = uint8(min(max(disparity,0),1).*255);
disparity = reshape(disparity,size(depth,1),size(depth,2));

% Compute height - define floor as 5 centimeters below the shelf bin or tote
[pixX,pixY] = meshgrid(1:640,1:480);
camX = (pixX-colorK(1,3)).*depth/colorK(1,1);
camY = (pixY-colorK(2,3)).*depth/colorK(2,2);
camZ = depth;
camPts = [camX(:),camY(:),camZ(:)]';
extCam2Bin = inv(extBin2World)*extCam2World;
camPts = extCam2Bin(1:3,1:3)*camPts+repmat(extCam2Bin(1:3,4),1,size(camPts,2));
if strcmp(environment,'shelf')
    zFloor = -0.05;
    zCeil = 0.20;
else
    zFloor = -0.05;
    zCeil = 0.20;
end
height = (camPts(3,:)-zFloor)./(zCeil-zFloor);
height = uint8(min(max(height,0),1).*255);
height(find(camZ == 0)) = 0;
height = reshape(height,size(depth,1),size(depth,2));

% Compute angle with gravity
addpath(genpath('HHA'));
[surfNorms,~] = computeNormalsSquareSupport(depth,(depth==0),3,1,colorK,ones(size(depth)));
extWorld2Cam = inv(extCam2World);
yDir = extWorld2Cam(1:3,1:3)*[0;0;-1];
yDir = yDir./norm(yDir);
angle = acosd(min(1,max(-1,sum(bsxfun(@times, surfNorms, reshape(yDir, 1, 1, 3)), 3))));
angle(find(depth==0)) = 0;

% Combine channels
HHA = cat(3,disparity,height,angle+128-90);

end

