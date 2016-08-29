function [objSegmPts,objSegmConf] = getSegmentedPointCloud(sceneData,frames,objMasks,segConfMaps)

% Create segmented object point cloud 
objSegmPts = [];
objSegmConf = [];
for frameIdx = frames
    tmpObjMask = objMasks{frameIdx};
    tmpSegConfMap = segConfMaps{frameIdx};
    tmpDepth = sceneData.depthFrames{frameIdx};
    tmpExtCam2World = sceneData.extCam2World{frameIdx};
    tmpExtCam2Bin = sceneData.extWorld2Bin * tmpExtCam2World;
    
    % Apply segmentation mask to depth image and project to camera space
    tmpDepth = tmpDepth.*double(tmpObjMask);
    [pixX,pixY] = meshgrid(1:640,1:480);
    camX = (pixX-sceneData.colorK(1,3)).*tmpDepth/sceneData.colorK(1,1);
    camY = (pixY-sceneData.colorK(2,3)).*tmpDepth/sceneData.colorK(2,2);
    camZ = tmpDepth;
    validDepth = find((camZ > 0.1) & (camZ < 1));
    camPts = [camX(validDepth),camY(validDepth),camZ(validDepth)]';
    camPts = tmpExtCam2Bin(1:3,1:3) * camPts + repmat(tmpExtCam2Bin(1:3,4),1,size(camPts,2));
    objSegmPts = [objSegmPts,camPts];
    objSegmConf = [objSegmConf,tmpSegConfMap(validDepth)'];
end

end

