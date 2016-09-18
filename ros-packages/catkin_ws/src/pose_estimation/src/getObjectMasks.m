function [objMasks,segmThresh,segmConfMaps] = getObjectMasks(scenePath,objName,frames)
% Take in segmentation confidence maps, compute a confidence threshold, and
% return binary masks
%
% function [objMasks,segmThresh,segmConfMaps] = getObjectMasks(dataPath,objName,frames)
% Input:
%   scenePath   - file path to the folder holding RGB-D data of the scene, 
%                 which also has segmentation results from marvin_convnet
%   objName     - name of the target object (aka. object ID)
%   frames      - 1xN array indicating which frames of the RGB-D sequence
%                 from the scene to use
% Output:
%   objMasks     - 1xN cell array of 480x640 binary object masks (computed
%                  from segmentation)
%   segmThresh   - segmentation confidence threshold
%   segmConfMaps - 1xN cell array of 480x640 confidence values from
%                  segmentation
%
% ---------------------------------------------------------
% Copyright (c) 2016, Andy Zeng
% 
% This file is part of the APC Vision Toolbox and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------

% Search through mask files in data directory
maskFiles = dir(fullfile(scenePath,'masks',sprintf('*.%s.mask.png',objName)));

% Load segmentation confidence maps
objMasks = {};
segmConfMaps = {};
segSum = zeros(480,640);
for frameIdx = frames
    segConf = double(imread(fullfile(scenePath,'masks',maskFiles(frameIdx).name)))./65535;
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

