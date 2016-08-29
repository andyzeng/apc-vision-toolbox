function depth = readDepth(path)
% Reads a depth image (16-bit PNG, circularly bitshifted to the right by 3 
% bits, depth in decimillimeters).
%
% function depth = readDepth(path)
% Input:
%   path  - file path to the 480x640 depth image
% Output:
%   depth - 480x640 float array of depth values in meters
%
% ---------------------------------------------------------
% Copyright (c) 2016, Andy Zeng
% 
% This file is part of the APC Vision Toolbox and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------

data = imread(fullfile(path));
data = bitor(bitshift(data,13),bitshift(data,-3));
depth = double(data)./10000;
