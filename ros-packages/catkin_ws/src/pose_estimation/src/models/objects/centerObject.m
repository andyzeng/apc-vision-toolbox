clear all; close all;

% Load object
% objCloud = offLoader('data/models/book_joke.off');
objName = 'dasani_water_bottle';
objCloud = pcread(sprintf('%s.ply',objName));

% % Downsample object
% objCloud = pcdownsample(objCloud,'gridAverage',0.001);
% objCloud = pcdenoise(objCloud,'NumNeighbors',4);

% Rotate with respect to PCA, major axis = x
% coeff = pca(objCloud.vmat');
% objCloud.vmat = inv(coeff) * objCloud.vmat;
objCloudDownsampled = pcdownsample(objCloud,'gridAverage',0.005);
objPts = objCloud.Location';
coeff = pca(objCloudDownsampled.Location);
objPts = inv(coeff) * objPts;

% % Make z-up for now
% objCloud.vmat = [0 0 1;0 1 0;1 0 0] * objCloud.vmat;

% Center object
objLimits = [min(objPts,[],2),max(objPts,[],2)];
objCenter = mean(objLimits,2);
objPts = objPts - repmat(objCenter,1,size(objPts,2));

% % Fix normals
% newFmat = objCloud.fmat;
% newFmat(2,:) = objCloud.fmat(3,:);
% newFmat(3,:) = objCloud.fmat(2,:);
% objCloud.fmat = newFmat;

% % Create artificial bottom for glue bottle
% objLimits = [min(objCloud.vmat,[],2),max(objCloud.vmat,[],2)];
% bottomPts = find(objCloud.vmat(3,:) < (objLimits(3,1) + 0.001));
% bottomPts = bottomPts(randsample(size(bottomPts,2),50));
% [triGridX,triGridY,triGridZ] = meshgrid(1:size(bottomPts,2),1:size(bottomPts,2),1:size(bottomPts,2));
% triComb = [bottomPts(triGridX(:));bottomPts(triGridY(:));bottomPts(triGridZ(:));bottomPts(triGridX(:))];
% objCloud.fmat = [objCloud.fmat triComb-1];

% Save object
% offWriter('data/models/book_joke_new.off',objCloud);
objCloud = pointCloud(objPts','Color',objCloud.Color);
pcwrite(objCloud,objName,'PLYformat','binary');
























