function [objMasks,segmThresh,segmConfMaps] = getObjectMasks(dataPath,objName,frames)

% Search through mask files in data directory
maskFiles = dir(fullfile(dataPath,'masks',sprintf('*.%s.mask.png',objName)));

% Load segmentation confidence maps
objMasks = {};
segmConfMaps = {};
segSum = zeros(480,640);
for frameIdx = frames
    segConf = double(imread(fullfile(dataPath,'masks',maskFiles(frameIdx).name)))./65535;
    segSum = segSum + segConf;
    segmConfMaps{frameIdx} = segConf;
end

% Compute segmentation threshold
numStd = 3;
numVals = 640*480*length(frames);
meanSegVal = 1/numVals*sum(sum(segSum));
segSum = zeros(480,640);
for frameIdx = frames
  segSum = segSum + (segmConfMaps{frameIdx}-meanSegVal).^2;
end
stdSegVal = sqrt((1/(numVals-1))*(sum(sum(segSum))));
segmThresh = meanSegVal + stdSegVal * numStd;
while segmThresh > 1
  numStd = numStd - 1;
  segmThresh = meanSegVal + stdSegVal * numStd;
end

% Use segmentation threshold to generate object segmentation masks
for frameIdx = frames
    segConf = segmConfMaps{frameIdx};
    objMasks{frameIdx} = segConf > segmThresh;
end

end

