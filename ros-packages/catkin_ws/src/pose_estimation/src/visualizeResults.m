function resultImage = visualizeResults(RtPCA,latentPCA,surfaceCentroid,surfaceRange,finalRt,finalScore,scenePath,objName,instanceIdx,sceneData,objSegPts,objMasks,objModelPts)

% if strcmp(objName,'rolodex_jumbo_pencil_cup') ||

% Show segmentation result (denoised)
if strcmp(sceneData.env,'tote')
  frames = [5 14];
else
  frames = [13];
end

allObjImage = [];
displayMode = 'Predicted Rigid Object Pose';
for baseFrameIdx = frames
  segImage = sceneData.colorFrames{baseFrameIdx};
  extBin2Cam = inv(sceneData.extCam2World{baseFrameIdx})*sceneData.extBin2World;
  if strcmp(objName,'rolodex_jumbo_pencil_cup') || strcmp(objName,'dasani_water_bottle') || strcmp(objName,'platinum_pets_dog_bowl') || strcmp(objName,'easter_turtle_sippy_cup')
    segMask = double(objMasks{baseFrameIdx})*255;
  else
    camObjSegPts = extBin2Cam(1:3,1:3) * objSegPts + repmat(extBin2Cam(1:3,4),1,size(objSegPts,2)); 
    pixX = round(((camObjSegPts(1,:).*sceneData.colorK(1,1))./camObjSegPts(3,:))+sceneData.colorK(1,3));
    pixY = round(((camObjSegPts(2,:).*sceneData.colorK(2,2))./camObjSegPts(3,:))+sceneData.colorK(2,3));
    validPix = find(pixX > 0 & pixX <= 640 & pixY > 0 & pixY <= 480);
    segMask = zeros(480,640);
    segMask(sub2ind(size(segMask),pixY(validPix)',pixX(validPix)')) = 255;
  end

  segImageR = double(segImage(:,:,1));
  segImageG = double(segImage(:,:,2));
  segImageB = double(segImage(:,:,3));
  segImageR(find(segMask)) = segImageR(find(segMask))./2;
  segImageG(find(segMask)) = (segImageG(find(segMask)) + 255)./2;
  segImageB(find(segMask)) = segImageB(find(segMask))./2;
  segImage(:,:,1) = uint8(round(segImageR));
  segImage(:,:,2) = uint8(round(segImageG));
  segImage(:,:,3) = uint8(round(segImageB));

  % finalRt
  % objModelPts

    drawBbox = true;
    drawModel = true;

  modelRt = extBin2Cam * sceneData.extWorld2Bin * finalRt;
  RtPCA = sceneData.extWorld2Bin * RtPCA;
  if strcmp(objName,'cherokee_easy_tee_shirt') || strcmp(objName,'kyjen_squeakin_eggs_plush_puppies') || strcmp(objName,'womens_knit_gloves') || strcmp(objName,'cloud_b_plush_bear') || ...
     strcmp(objName,'scotch_bubble_mailer')
    modelRt(1:3,4) = surfaceCentroid;
    modelRt(1:3,1:3) = RtPCA(1:3,1:3);

    extBin2Obj = inv(sceneData.extWorld2Bin * modelRt);
    objModelPts = extBin2Obj(1:3,1:3) * objSegPts + repmat(extBin2Obj(1:3,4),1,size(objSegPts,2));

    modelRt = extBin2Cam * sceneData.extWorld2Bin * modelRt;

    drawBbox = false;
    displayMode = 'Estimated Observable Surface PCA';

    axisRange = [latentPCA(1)*50,latentPCA(2)*50,latentPCA(3)*50];
  elseif strcmp(objName,'rolodex_jumbo_pencil_cup')
    drawBbox = false;
    drawModel = false;
    axisRange = [0.06,0.06,0.06];
  elseif strcmp(objName,'dasani_water_bottle')
    drawBbox = false;
    drawModel = false;
    axisRange = [0.1,0.03,0.03];
  elseif strcmp(objName,'platinum_pets_dog_bowl')
    drawBbox = false;
    drawModel = false;
    axisRange = [0.06,0.06,0.015];
  else
    axisRange = [max(objModelPts(1,:)),max(objModelPts(2,:)),max(objModelPts(3,:))];
  end

  camObjModelPts = modelRt(1:3,1:3) * objModelPts + repmat(modelRt(1:3,4),1,size(objModelPts,2));
  pixX = round(((camObjModelPts(1,:).*sceneData.colorK(1,1))./camObjModelPts(3,:))+sceneData.colorK(1,3));
  pixY = round(((camObjModelPts(2,:).*sceneData.colorK(2,2))./camObjModelPts(3,:))+sceneData.colorK(2,3));
  validPix = find(pixX > 0 & pixX <= 640 & pixY > 0 & pixY <= 480);
  objMask = zeros(480,640);
  objMask(sub2ind(size(objMask),pixY(validPix)',pixX(validPix)')) = 1;

  objImage = sceneData.colorFrames{baseFrameIdx};
  if drawModel
    objImageR = double(objImage(:,:,1));
    objImageG = double(objImage(:,:,2));
    objImageB = double(objImage(:,:,3));
    objImageR(find(objMask)) = objImageR(find(objMask))./2;
    objImageG(find(objMask)) = (objImageG(find(objMask)) + 255)./2;
    objImageB(find(objMask)) = objImageB(find(objMask))./2;
    objImage(:,:,1) = uint8(round(objImageR));
    objImage(:,:,2) = uint8(round(objImageG));
    objImage(:,:,3) = uint8(round(objImageB));
  end

  axisPts = [[0;0;0],[axisRange(1);0;0],[0;axisRange(2);0],[0;0;axisRange(3)]];
  camAxisPts = modelRt(1:3,1:3) * axisPts + repmat(modelRt(1:3,4),1,size(axisPts,2));
  axisPixX = round(((camAxisPts(1,:).*sceneData.colorK(1,1))./camAxisPts(3,:))+sceneData.colorK(1,3));
  axisPixY = round(((camAxisPts(2,:).*sceneData.colorK(2,2))./camAxisPts(3,:))+sceneData.colorK(2,3));
  axisPts2D = [axisPixX; axisPixY];
  canvasImage = insertShape(objImage, 'line', [axisPts2D(:,1)',axisPts2D(:,2)'], 'LineWidth', 3,'Color', 'red');
  canvasImage = insertShape(canvasImage, 'line', [axisPts2D(:,1)',axisPts2D(:,3)'], 'LineWidth', 3,'Color', 'green');
  canvasImage = insertShape(canvasImage, 'line', [axisPts2D(:,1)',axisPts2D(:,4)'], 'LineWidth', 3,'Color', 'blue');
  % test = insertShape(test, 'FilledCircle', [axisPts2D(:,1)' 2], 'LineWidth', 1,'Opacity',1,'Color', 'black');

  if drawBbox
    bboxRange = [min(objModelPts(1,:)),max(objModelPts(1,:));min(objModelPts(2,:)),max(objModelPts(2,:));min(objModelPts(3,:)),max(objModelPts(3,:))];
    bboxCorners = [[bboxRange(1,1);bboxRange(2,1);bboxRange(3,1)], ...
                   [bboxRange(1,1);bboxRange(2,1);bboxRange(3,2)], ...
                   [bboxRange(1,1);bboxRange(2,2);bboxRange(3,2)], ...
                   [bboxRange(1,1);bboxRange(2,2);bboxRange(3,1)], ...
                   [bboxRange(1,2);bboxRange(2,1);bboxRange(3,1)], ...
                   [bboxRange(1,2);bboxRange(2,1);bboxRange(3,2)], ...
                   [bboxRange(1,2);bboxRange(2,2);bboxRange(3,2)], ...
                   [bboxRange(1,2);bboxRange(2,2);bboxRange(3,1)]];
    camCornerPts = modelRt(1:3,1:3) * bboxCorners + repmat(modelRt(1:3,4),1,size(bboxCorners,2));
    cornerPixX = round(((camCornerPts(1,:).*sceneData.colorK(1,1))./camCornerPts(3,:))+sceneData.colorK(1,3));
    cornerPixY = round(((camCornerPts(2,:).*sceneData.colorK(2,2))./camCornerPts(3,:))+sceneData.colorK(2,3));
    cornerPts2D = [cornerPixX; cornerPixY];
    if max(cornerPts2D(:)) < 1500  % to prevent bad detection to make drawing too slow
      canvasImage = insertShape(canvasImage, 'line', [cornerPts2D(:,1)',cornerPts2D(:,2)';cornerPts2D(:,2)',cornerPts2D(:,3)';cornerPts2D(:,3)',cornerPts2D(:,4)';cornerPts2D(:,4)',cornerPts2D(:,1)'], 'LineWidth', 1,'Color', 'yellow');
      canvasImage = insertShape(canvasImage, 'line', [cornerPts2D(:,5)',cornerPts2D(:,6)';cornerPts2D(:,6)',cornerPts2D(:,7)';cornerPts2D(:,7)',cornerPts2D(:,8)';cornerPts2D(:,8)',cornerPts2D(:,5)'], 'LineWidth', 1,'Color', 'yellow');
      canvasImage = insertShape(canvasImage, 'line', [cornerPts2D(:,1)',cornerPts2D(:,5)';cornerPts2D(:,2)',cornerPts2D(:,6)';cornerPts2D(:,3)',cornerPts2D(:,7)';cornerPts2D(:,4)',cornerPts2D(:,8)'], 'LineWidth', 1,'Color', 'yellow');
    end
  end
  
  if baseFrameIdx == frames(end);
    segImage = insertText(segImage,[10 450],'Segmentation Results','TextColor','white','BoxColor','black');
    canvasImage = insertText(canvasImage,[10 450],sprintf('%s',displayMode),'TextColor','white','BoxColor','black');
  end
  
  paddedSegImage = uint8(zeros(size(segImage,1)+5,size(segImage,2)+5,size(segImage,3)));
  paddedSegImage(1:480,1:640,:) = segImage;
  segImage = paddedSegImage;
  paddedCanvasImage = uint8(zeros(size(canvasImage,1)+5,size(canvasImage,2)+5,size(canvasImage,3)));
  paddedCanvasImage(1:480,6:645,:) = canvasImage;
  canvasImage = paddedCanvasImage;
  
  allObjImage = cat(1,allObjImage,cat(2,segImage,canvasImage));
end

paddedAllObjImage = uint8(zeros(size(allObjImage,1)+5,size(allObjImage,2)+10,size(allObjImage,3)));
paddedAllObjImage(6:end,6:(end-5),:) = allObjImage;
allObjImage = paddedAllObjImage;
% objImage = canvasImage;

% resultImage = objImage;



% imshow(test)

             
imwrite(allObjImage,fullfile(scenePath,strcat(objName,sprintf('.%d.result.png',instanceIdx))));



end

