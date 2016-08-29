function [N b] = computeNormalsSquareSupport(depthImage, missingMask, R, sc, cameraMatrix, superpixels)
% function [N b] = computeNormalsMatlab(depthImage, missingMask, R, sc, cameraMatrix, superpixels)
%   Clip out a 2R+1 x 2R+1 window at each point and estimate 
%   the normal from points within this window. In case the window 
%   straddles more than a single superpixel, only take points in the 
%   same superpixel as the centre pixel. Takes about 0.5 second per image.
%   Does not use ImageStack so is more platform independent, but 
%   gives different results.
% Input: 
%   depthImage:   in metres
%   missingMask:  boolean mask of what data was missing
%   R:            radius of clipping
%   sc:           to upsample or not
%   superpixels:  superpixel map to define bounadaries that should
%                 not be straddled
%
% Output: The normal at pixel (x,y) is N(x, y, :)'pt + b(x,y) = 0
%   N:            Normal field
%   b:            bias

% AUTORIGHTS
% ---------------------------------------------------------
% Copyright (c) 2014, Saurabh Gupta
% 
% This file is part of the RGBD Utils code and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------
  
  % Convert to centi metres
	depthImage = depthImage*100;
	[X, Y, Z] = getPointCloudFromZ(depthImage, cameraMatrix, sc);
  Xf = X; Yf = Y; Zf = Z; % Only used for assigning the sign to N
  X(missingMask) = NaN;
  Y(missingMask) = NaN;
  Z(missingMask) = NaN;
	
  N = NaN([size(Z), 3]);
	
  one_Z = 1./Z; 
	X_Z = X./Z;
	Y_Z = Y./Z;
	one = Z; one(~isnan(one)) = 1;
	X_ZZ = X./(Z.*Z);
	Y_ZZ = Y./(Z.*Z);

	AtARaw = cat(3, X_Z.^2, X_Z.*Y_Z, X_Z, Y_Z.^2, Y_Z, one);
	AtbRaw = cat(3, X_ZZ, Y_ZZ, one_Z);

	% With clipping
	AtA = filterItChopOff(cat(3, AtARaw, AtbRaw), R, superpixels);
	Atb = AtA(:, :, (size(AtARaw,3)+1):end);
	AtA = AtA(:, :, 1:size(AtARaw,3));

	[AtA_1 detAtA] = invertIt(AtA);

	N = mutiplyIt(AtA_1, Atb);
	b = N(:,:,1);
	b(:) = -detAtA;
	b = bsxfun(@rdivide, b, sqrt(sum(N.^2,3)));
	N = bsxfun(@rdivide, N, sqrt(sum(N.^2,3)));

	%Reorient the normals to point out from the scene.
	SN = sign(N(:,:,3));
	SN(SN == 0) = 1;
	N = N.*repmat(SN, [1 1 3]);
	b = b.*SN;
	sn = sign(sum(N.*cat(3, Xf, Yf, Zf),3));
	sn(isnan(sn)) = 1;
	sn(sn == 0) = 1;
  % sn(:) = 1;
	N = repmat(sn,[1 1 3]).*N;
	b = b.*sn;
end

function x = mutiplyIt(AtA_1, Atb)
	a = @(k) AtA_1(:,:,k);
	b = @(k) Atb(:,:,k);
	x1 = a(1).*b(1) + a(2).*b(2) + a(3).*b(3);
	x2 = a(2).*b(1) + a(4).*b(2) + a(5).*b(3);
	x3 = a(3).*b(1) + a(5).*b(2) + a(6).*b(3);
	x = cat(3, x1, x2, x3);
end

function [AtA_1 detAtA]= invertIt(AtA)
	a = @(k) AtA(:,:,k);

	A = a(4).*a(6) - a(5).*a(5);
	D = -(a(2).*a(6 )- a(3).*a(5));
	G = a(2).*a(5) - a(3).*a(4);
	E = a(1).*a(6) - a(3).*a(3);
	H = -(a(1).*a(5) - a(2).*a(3));
	K = a(1).*a(4) - a(2).*a(2);

	detAtA = a(1).*A + a(2).*D + a(3).*G;

	AtA_1 = cat(3, A, D, G, E, H, K);
	%AtA_1 = bsxfun(@rdivide, AtA_1, detAtA);
end

function fFilt = filterItChopOff(f, r, sp)
	f(isnan(f)) = 0;
	[H W d] = size(f);
	B = ones(2*r+1,2*r+1);
	
	minSP = ordfilt2(sp, 1, B, 'symmetric');
	maxSP = ordfilt2(sp, numel(B), B, 'symmetric');
	ind = find(minSP(:) ~= sp(:) | maxSP(:) ~= sp(:));
	spInd = sp; spInd(:) = 1:numel(sp);

	delta = zeros(size(f));
	delta = reshape(delta, [H*W, d]);
	f = reshape(f, [H*W, d]);

	% Calculate the delta...
	% fprintf('Need to recompute for %d/%d...\n', length(ind), numel(sp));
	[I, J] = ind2sub([H W],  ind);
	for i = 1:length(ind),
		x = I(i); y = J(i);
		clipInd = spInd(max(1, x-r):min(H, x+r), max(1, y-r):min(W, y+r));
		diffInd = clipInd(sp(clipInd) ~= sp(x,y));
		delta(ind(i), :) = sum(f(diffInd, :),1);
	end

	delta = reshape(delta, [H W d]);
	f = reshape(f, [H W d]);	
	
  for i = 1:size(f,3),
    fFilt(:,:,i) = filter2(B, f(:,:,i));
  end
	fFilt = fFilt-delta;
end

function fFilt = filterIt(f, r)
	B = ones(2*r+1,2*r+1);
	f(isnan(f)) = 0;
	for i = 1:size(f,3),
		fFilt(:,:,i) = filter2(B, f(:,:,i));
	end
end