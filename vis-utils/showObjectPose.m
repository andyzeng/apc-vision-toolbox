function [canvasSeg,canvasPose] = dispObjPose(objName,canvasSeg,canvasPose,colorFrame,depthFrame,extCam2World,K,objCloud,objPoseWorld,objColor)
%DISPOBJPOSE Summary of this function goes here
%   Detailed explanation goes here

% Compute object pose in camera coordinates
objPoseCam = inv(extCam2World)*objPoseWorld;

% Compute projected object model
objPts = objCloud.Location';
objPtsCam = objPoseCam(1:3,1:3)*objPts+repmat(objPoseCam(1:3,4),1,size(objPts,2));
objPixX = round(((objPtsCam(1,:).*K(1,1))./objPtsCam(3,:))+K(1,3));
objPixY = round(((objPtsCam(2,:).*K(2,2))./objPtsCam(3,:))+K(2,3));
validObjPix = find(objPixX > 0 & objPixX <= 640 & objPixY > 0 & objPixY <= 480);
objPixX = objPixX(validObjPix);
objPixY = objPixY(validObjPix);
objMask = zeros(480,640);
objMask(sub2ind(size(objMask),objPixY',objPixX')) = 1;

if strcmp(objName,'cherokee_easy_tee_shirt')
    drawObj = false; drawAxis = true; drawBbox = false;
else
    drawObj = true; drawAxis = false; drawBbox = true;
end

% Compute object bounding box
bboxRange = [min(objPts(1,:)),max(objPts(1,:));min(objPts(2,:)),max(objPts(2,:));min(objPts(3,:)),max(objPts(3,:))];
bboxCorners = [[bboxRange(1,1);bboxRange(2,1);bboxRange(3,1)], ...
               [bboxRange(1,1);bboxRange(2,1);bboxRange(3,2)], ...
               [bboxRange(1,1);bboxRange(2,2);bboxRange(3,2)], ...
               [bboxRange(1,1);bboxRange(2,2);bboxRange(3,1)], ...
               [bboxRange(1,2);bboxRange(2,1);bboxRange(3,1)], ...
               [bboxRange(1,2);bboxRange(2,1);bboxRange(3,2)], ...
               [bboxRange(1,2);bboxRange(2,2);bboxRange(3,2)], ...
               [bboxRange(1,2);bboxRange(2,2);bboxRange(3,1)]];
cornerPtsCam = objPoseCam(1:3,1:3) * bboxCorners + repmat(objPoseCam(1:3,4),1,size(bboxCorners,2));
cornerPixX = round(((cornerPtsCam(1,:).*K(1,1))./cornerPtsCam(3,:))+K(1,3));
cornerPixY = round(((cornerPtsCam(2,:).*K(2,2))./cornerPtsCam(3,:))+K(2,3));
cornerPts2D = [cornerPixX; cornerPixY];

% Find depth points inside object bounding box
if false % Disabled
    [depthPixX,depthPixY] = meshgrid(1:640,1:480);
    camX = (depthPixX-K(1,3)).*depthFrame/K(1,1);
    camY = (depthPixY-K(2,3)).*depthFrame/K(2,2);
    camZ = depthFrame;
    camPts = [camX(:),camY(:),camZ(:)]';
    extWorld2Obj = inv(objPoseWorld);
    extCam2Obj =  extWorld2Obj*extCam2World;
    camPtsObj = extCam2Obj(1:3,1:3) * camPts + repmat(extCam2Obj(1:3,4),1,size(camPts,2));
    validCamIdx = find(camPtsObj(1,:) > bboxRange(1,1) & camPtsObj(1,:) < bboxRange(1,2) & ...
                       camPtsObj(2,:) > bboxRange(2,1) & camPtsObj(2,:) < bboxRange(2,2) & ...
                       camPtsObj(3,:) > bboxRange(3,1) & camPtsObj(3,:) < bboxRange(3,2));
    segObjMask = zeros(480,640);
    segObjMask(validCamIdx) = 1;
    objMask = objMask | segObjMask;
end

% Fill holes
for fillIdx = 1:20
    objMask = imfill(objMask);
end

% Draw projected object + segmentation
if drawObj
    canvasR = double(canvasSeg(:,:,1));
    canvasG = double(canvasSeg(:,:,2));
    canvasB = double(canvasSeg(:,:,3));
    canvasR(find(objMask)) = objColor(1); %(frameR(find(objMask))+objColor(1))./2;
    canvasG(find(objMask)) = objColor(2); %(frameG(find(objMask))+objColor(2))./2;
    canvasB(find(objMask)) = objColor(3); %(frameB(find(objMask))+objColor(3))./2;
    canvasSeg(:,:,1) = uint8(round(canvasR));
    canvasSeg(:,:,2) = uint8(round(canvasG));
    canvasSeg(:,:,3) = uint8(round(canvasB));
    frameR = double(colorFrame(:,:,1));
    frameG = double(colorFrame(:,:,2));
    frameB = double(colorFrame(:,:,3));
    canvasR = double(canvasPose(:,:,1));
    canvasG = double(canvasPose(:,:,2));
    canvasB = double(canvasPose(:,:,3));
    canvasR(find(objMask)) = (frameR(find(objMask))+objColor(1))./2;
    canvasG(find(objMask)) = (frameG(find(objMask))+objColor(2))./2;
    canvasB(find(objMask)) = (frameB(find(objMask))+objColor(3))./2;
    canvasPose(:,:,1) = uint8(round(canvasR));
    canvasPose(:,:,2) = uint8(round(canvasG));
    canvasPose(:,:,3) = uint8(round(canvasB));
end

% Draw object axis
if drawAxis
    axisRange = [max(objPts(1,:)),max(objPts(2,:)),max(objPts(3,:))];     
    axisRange = [0.05,0.05,0.05];               
    axisPts = [[0;0;0],[axisRange(1);0;0],[0;axisRange(2);0],[0;0;axisRange(3)]];
    camAxisPts = objPoseCam(1:3,1:3) * axisPts + repmat(objPoseCam(1:3,4)+[0;0;0.05],1,size(axisPts,2));
    axisPixX = round(((camAxisPts(1,:).*K(1,1))./camAxisPts(3,:))+K(1,3));
    axisPixY = round(((camAxisPts(2,:).*K(2,2))./camAxisPts(3,:))+K(2,3));
    axisPts2D = [axisPixX; axisPixY];
    canvasPose = insertShape(canvasPose, 'line', [axisPts2D(:,1)',axisPts2D(:,2)'], 'LineWidth', 5,'Color', objColor);
    canvasPose = insertShape(canvasPose, 'line', [axisPts2D(:,1)',axisPts2D(:,3)'], 'LineWidth', 5,'Color', objColor);
    canvasPose = insertShape(canvasPose, 'line', [axisPts2D(:,1)',axisPts2D(:,4)'], 'LineWidth', 5,'Color', objColor);                
end
    
% Draw object bounding box
if drawBbox
    [~,furthestCornerIdx] = max(cornerPtsCam(3,:));
    connections = [1,2;2,3;3,4;4,1;5,6;6,7;7,8;8,5;1,5;2,6;3,7;4,8];
    for connIdx = 1:size(connections,1)
        if ismember(furthestCornerIdx,connections(connIdx,:))
            canvasPose = insertShape(canvasPose,'line',[cornerPts2D(:,connections(connIdx,1))',cornerPts2D(:,connections(connIdx,2))'],'LineWidth',5,'Color',objColor); % (TODO: add dotted lines to the occluded parts of the bounding box)
        else
            canvasPose = insertShape(canvasPose,'line',[cornerPts2D(:,connections(connIdx,1))',cornerPts2D(:,connections(connIdx,2))'],'LineWidth',5,'Color',objColor);
        end
    end
end

end

