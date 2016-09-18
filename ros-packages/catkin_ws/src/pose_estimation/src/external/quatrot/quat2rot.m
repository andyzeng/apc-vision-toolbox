function R = quat2rotm( q )
%QUAT2ROTM Convert quaternion to rotation matrix
%   R = QUAT2ROTM(Q) converts a unit quaternion, Q, into an orthonormal
%   rotation matrix, R. The input, Q, is an N-by-4 matrix containing N quaternions. 
%   Each quaternion represents a 3D rotation and is of the form q = [w x y z], 
%   with a scalar number as the first value. Each element of Q must be a real number.
%   The output, R, is an 3-by-3-by-N matrix containing N rotation matrices.
%
%   Example:
%      % Convert a quaternion to rotation matrix
%      q = [0.7071 0.7071 0 0];
%      R = quat2rotm(q)
%
%   See also rotm2quat

%   Copyright 2014-2015 The MathWorks, Inc.

%#codegen

% robotics.internal.validation.validateNumericMatrix(q, 'quat2rotm', 'q', ...
%     'ncols', 4);

% Normalize and transpose the quaternions
q = robotics.internal.normalizeRows(q).';

% Reshape the quaternions in the depth dimension
q2 = reshape(q,[4 1 size(q,2)]);

s = q2(1,1,:);
x = q2(2,1,:);
y = q2(3,1,:);
z = q2(4,1,:);

% Explicitly define concatenation dimension for codegen
tempR = cat(1, 1 - 2*(y.^2 + z.^2),   2*(x.*y - s.*z),   2*(x.*z + s.*y),...
2*(x.*y + s.*z), 1 - 2*(x.^2 + z.^2),   2*(y.*z - s.*x),...
2*(x.*z - s.*y),   2*(y.*z + s.*x), 1 - 2*(x.^2 + y.^2) );

R = reshape(tempR, [3, 3, length(s)]);
R = permute(R, [2 1 3]);

end

