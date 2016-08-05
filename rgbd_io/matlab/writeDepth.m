function writeDepth(depth,path)
% WRITEDEPTH: Saves a depth image (16-bit PNG, circularly bitshifted to 
% the right by 3 bits, depth in decimillimeters).
%
% Arguments: (input)
%   depth - 480x640 float array of depth values in meters
%   path  - file path and name to the 480x640 depth image
%
% Author:  Andy Zeng, andyz@princeton.edu

data = uint16(round(depth.*10000));
data = bitor(bitshift(data,3),bitshift(data,-13));
imwrite(data,fullfile(path));
