function [x3 y3 z3] = getPointCloudFromZ(Z, C, s)
%function [x3 y3 z3] = getPointCloudFromZ(Z, C, s)
% Input: 
%   Z, C the camera matrix, s is the factor by which Z has been upsampled!?
% Output: 
%   the point cloud in x3, y3, z3;

% AUTORIGHTS
% ---------------------------------------------------------
% Copyright (c) 2014, Saurabh Gupta
% 
% This file is part of the RGBD Utils code and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------

	if(~exist('s','var')), s = 1; end

	[H, W, gr] = size(Z);
  [xx, yy] = meshgrid(1:W,1:H);

 	%% Color camera parameters
	cc_rgb = C(1:2,3)*s;
	fc_rgb = diag(C(1:2,1:2))*s;

	x3 = (xx - cc_rgb(1)) .* Z/ fc_rgb(1);
	y3 = (yy - cc_rgb(2)) .* Z / fc_rgb(2);
	z3 = Z;
end