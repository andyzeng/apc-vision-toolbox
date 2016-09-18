function objSegmPts = denoisePointCloud(objSegmPts)
% Remove outlier points from the principal components (computed from PCA)
% of a noisy point cloud
%
% function objSegPts = denoisePointCloud(objSegPts)
% Input:
%   objSegPts - 3xN float array of 3D points
% Output:
%   objSegPts - 3xN float array of 3D points
%
% ---------------------------------------------------------
% Copyright (c) 2016, Andy Zeng
% 
% This file is part of the APC Vision Toolbox and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------

% Compute PCA for removing outliers from segmented point cloud
coeffPCAoutlier = pca(objSegmPts');
currObjSegPtsAligned = (inv(coeffPCAoutlier) * (objSegmPts - repmat(median(objSegmPts,2),1,size(objSegmPts,2))))';

% Find outliers from first principal component (local minima < 5%, maxima > 50%)
currObjSegPtsDistPC1 = currObjSegPtsAligned(:,1);
sortedObjSegPtsDistPC1 = sort(currObjSegPtsDistPC1);
histCountsDistPC1 = histcounts(sortedObjSegPtsDistPC1,min(sortedObjSegPtsDistPC1):0.01:max(sortedObjSegPtsDistPC1));
histCountsDistPC1 = conv(histCountsDistPC1,[1/3 1/3 1/3]);
midBinDistPC1 = ceil(0-min(sortedObjSegPtsDistPC1)/0.01);
[PC1MaxPeaks,PC1MaxLocs] = findpeaks(histCountsDistPC1);
PC1MaxLocs = PC1MaxLocs(find(PC1MaxPeaks > max(PC1MaxPeaks)*0.3));
[dist2Peak,nearestPeakIdx] = min(abs(PC1MaxLocs - midBinDistPC1));
midBinDistPC1 = PC1MaxLocs(nearestPeakIdx);
[PC1Peaks,PC1MinLocs] = findpeaks(max(histCountsDistPC1) - histCountsDistPC1);
PC1MinLocs = PC1MinLocs(find(max(histCountsDistPC1) - PC1Peaks < 0.05*max(histCountsDistPC1)));
PC1Range = [max([1,PC1MinLocs(find(PC1MinLocs < midBinDistPC1))]),min([length(histCountsDistPC1),PC1MinLocs(find(PC1MinLocs > midBinDistPC1))])];
PC1Range = (min(sortedObjSegPtsDistPC1)-0.005 + (PC1Range-1)*0.01);

% Find outliers from second principle component
currObjSegPtsDistPC2 = currObjSegPtsAligned(:,2);
sortedObjSegPtsDistPC2 = sort(currObjSegPtsDistPC2);
histCountsDistPC2 = histcounts(sortedObjSegPtsDistPC2,min(sortedObjSegPtsDistPC2):0.01:max(sortedObjSegPtsDistPC2));
histCountsDistPC2 = conv(histCountsDistPC2,[1/3 1/3 1/3]);
midBinDistPC2 = ceil(0-min(sortedObjSegPtsDistPC2)/0.01);
[PC2MaxPeaks,PC2MaxLocs] = findpeaks(histCountsDistPC2);
PC2MaxLocs = PC2MaxLocs(find(PC2MaxPeaks > max(PC2MaxPeaks)*0.3));
[dist2Peak,nearestPeakIdx] = min(abs(PC2MaxLocs - midBinDistPC2));
midBinDistPC2 = PC2MaxLocs(nearestPeakIdx);
[PC2Peaks,PC2MinLocs] = findpeaks(max(histCountsDistPC2) - histCountsDistPC2);
PC2MinLocs = PC2MinLocs(find(max(histCountsDistPC2) - PC2Peaks < 0.05*max(histCountsDistPC2)));
PC2Range = [max([1,PC2MinLocs(find(PC2MinLocs < midBinDistPC2))]),min([length(histCountsDistPC2),PC2MinLocs(find(PC2MinLocs > midBinDistPC2))])];
PC2Range = (min(sortedObjSegPtsDistPC2)-0.005 + (PC2Range-1)*0.01);

% Find outliers from third principle component
currObjSegPtsDistPC3 = currObjSegPtsAligned(:,3);
sortedObjSegPtsDistPC3 = sort(currObjSegPtsDistPC3);
histCountsDistPC3 = histcounts(sortedObjSegPtsDistPC3,min(sortedObjSegPtsDistPC3):0.01:max(sortedObjSegPtsDistPC3));
histCountsDistPC3 = conv(histCountsDistPC3,[1/3 1/3 1/3]);
midBinDistPC3 = ceil(0-min(sortedObjSegPtsDistPC3)/0.01);
[PC3MaxPeaks,PC3MaxLocs] = findpeaks(histCountsDistPC3);
PC3MaxLocs = PC3MaxLocs(find(PC3MaxPeaks > max(PC3MaxPeaks)*0.3));
[dist2Peak,nearestPeakIdx] = min(abs(PC3MaxLocs - midBinDistPC3));
midBinDistPC3 = PC3MaxLocs(nearestPeakIdx);
[PC3Peaks,PC3MinLocs] = findpeaks(max(histCountsDistPC3) - histCountsDistPC3);
PC3MinLocs = PC3MinLocs(find(max(histCountsDistPC3) - PC3Peaks < 0.05*max(histCountsDistPC3)));
PC3Range = [max([1,PC3MinLocs(find(PC3MinLocs < midBinDistPC3))]),min([length(histCountsDistPC3),PC3MinLocs(find(PC3MinLocs > midBinDistPC3))])];
PC3Range = (min(sortedObjSegPtsDistPC3)-0.005 + (PC3Range-1)*0.01);

% Remove outliers
objSegmPts = objSegmPts(:,find((currObjSegPtsDistPC1 > PC1Range(1)) & (currObjSegPtsDistPC1 < PC1Range(2)) & ...
                             (currObjSegPtsDistPC2 > PC2Range(1)) & (currObjSegPtsDistPC2 < PC2Range(2)) & ...
                             (currObjSegPtsDistPC3 > PC3Range(1)) & (currObjSegPtsDistPC3 < PC3Range(2))));

end

