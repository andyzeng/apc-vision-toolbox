function writeDepth(depth,path)
% Saves a depth image (16-bit PNG, circularly bitshifted to the right by 3 
% bits, depth in decimillimeters).
%
% function writeDepth(depth,path)
% Input:
%   depth - 480x640 float array of depth values in meters
%   path  - file path and name to the 480x640 depth image
%
% ---------------------------------------------------------
% Copyright (c) 2016, Andy Zeng
% 
% This file is part of the APC Vision Toolbox and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------

data = uint16(round(depth.*10000));
data = bitor(bitshift(data,3),bitshift(data,-13));
imwrite(data,fullfile(path));
