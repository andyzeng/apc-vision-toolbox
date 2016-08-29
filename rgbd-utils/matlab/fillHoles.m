function depth = fillHoles(depth)
% Fill 1-pixel width holes in depth image by averaging neighboring values
%
% function depth = fillHoles(depth)
% Input:
%   depth - 480x640 float array of depth values in meters
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

roiDepth = depth(2:(end-1),2:(end-1));

% Horizontal averaging
rightDepth = depth(2:(end-1),3:end);
leftDepth = depth(2:(end-1),1:(end-2));
horzIdx = find(roiDepth == 0 & leftDepth > 0 & rightDepth > 0);
interp = ((leftDepth+rightDepth)./2);
roiDepth(horzIdx) = roiDepth(horzIdx) + interp(horzIdx);

% Vertical averaging
upperDepth = depth(1:(end-2),2:(end-1));
lowerDepth = depth(3:end,2:(end-1));
vertIdx = find(roiDepth == 0 & lowerDepth > 0 & upperDepth > 0);
interp = ((lowerDepth+upperDepth)./2);
roiDepth(vertIdx) = roiDepth(vertIdx) + interp(vertIdx);

% Diagonal averaging 1 (upper left to lower right)
upperLeftDepth = depth(1:(end-2),1:(end-2));
lowerRightDepth = depth(3:end,3:end);
diag1Idx = find(roiDepth == 0 & lowerRightDepth > 0 & upperLeftDepth > 0);
interp = ((lowerRightDepth+upperLeftDepth)./2);
roiDepth(diag1Idx) = roiDepth(diag1Idx) + interp(diag1Idx);

% Diagonal averaging 2 (lower left to upper right)
lowerLeftDepth = depth(3:end,1:(end-2));
upperRightDepth = depth(1:(end-2),3:end);
diag2Idx = find(roiDepth == 0 & lowerLeftDepth > 0 & upperRightDepth > 0);
interp = ((lowerLeftDepth+upperRightDepth)./2);
roiDepth(diag2Idx) = roiDepth(diag2Idx) + interp(diag2Idx);

denom = ones(size(roiDepth));
denom([horzIdx;vertIdx;diag1Idx;diag2Idx]) = 0;
denom(horzIdx) = denom(horzIdx)+1;
denom(vertIdx) = denom(vertIdx)+1;
denom(diag1Idx) = denom(diag1Idx)+1;
denom(diag2Idx) = denom(diag2Idx)+1;
roiDepth = roiDepth./denom;
depth(2:(end-1),2:(end-1)) = roiDepth; 

% Vertical averaging on edges
roiEdges = [depth(2:(end-1),1),depth(2:(end-1),end)];
upperDepth = [depth(1:(end-2),1),depth(1:(end-2),end)];
lowerDepth = [depth(3:end,1),depth(3:end,end)];
vertIdx = find(roiEdges == 0 & lowerDepth > 0 & upperDepth > 0);
interp = ((lowerDepth+upperDepth)./2);
roiEdges(vertIdx) = interp(vertIdx);
depth(2:(end-1),1) = roiEdges(:,1); 
depth(2:(end-1),end) = roiEdges(:,2); 

% Horizontal averaging on edges
roiEdges = [depth(1,2:(end-1));depth(end,2:(end-1))];
rightDepth = [depth(1,3:end);depth(end,3:end)];
leftDepth = [depth(1,1:(end-2));depth(end,1:(end-2))];
horzIdx = find(roiEdges == 0 & leftDepth > 0 & rightDepth > 0);
interp = ((leftDepth+rightDepth)./2);
roiEdges(horzIdx) = interp(horzIdx);
depth(1,2:(end-1)) = roiEdges(1,:); 
depth(end,2:(end-1)) = roiEdges(2,:); 
end








