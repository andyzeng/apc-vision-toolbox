function depth = readDepth(path)
% READDEPTH: Reads a depth image (16-bit PNG, circularly bitshifted to the
% right by 3 bits, depth in decimillimeters).
%
% Arguments: (input)
%   path  - file path to the 480x640 depth image
%
% Arguments: (output)
%   depth - 480x640 float array of depth values in meters

data = imread(fullfile(path));
data = bitor(bitshift(data,13),bitshift(data,-3));
depth = double(data)./10000;
