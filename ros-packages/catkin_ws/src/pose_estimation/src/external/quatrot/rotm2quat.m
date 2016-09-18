function quat = rotm2quat( R )
%ROTM2QUAT Convert rotation matrix to quaternion
%   Q = ROTM2QUAT(R) converts a 3D rotation matrix, R, into the corresponding
%   unit quaternion representation, Q. The input, R, is an 3-by-3-by-N matrix
%   containing N orthonormal rotation matrices.
%   The output, Q, is an N-by-4 matrix containing N quaternions. Each
%   quaternion is of the form q = [w x y z], with a scalar number as
%   the first value. Each element of Q must be a real number.
%
%   If the input matrices are not orthonormal, the function will
%   return the quaternions that correspond to the orthonormal matrices
%   closest to the imprecise matrix inputs.
%
%
%   Example:
%      % Convert a rotation matrix to a quaternion
%      R = [0 0 1; 0 1 0; -1 0 0];
%      q = rotm2quat(R)
%
%   References:
%   [1] I.Y. Bar-Itzhack, "New method for extracting the quaternion from a 
%       rotation matrix," Journal of Guidance, Control, and Dynamics, 
%       vol. 23, no. 6, pp. 1085-1087, 2000
%
%   See also quat2rotm

%   Copyright 2014-2016 The MathWorks, Inc.

%#codegen

% robotics.internal.validation.validateRotationMatrix(R, 'rotm2quat', 'R');

% Pre-allocate output
quat = zeros(size(R,3), 4, 'like', R);

% Calculate all elements of symmetric K matrix
K11 = R(1,1,:) - R(2,2,:) - R(3,3,:);
K12 = R(1,2,:) + R(2,1,:);
K13 = R(1,3,:) + R(3,1,:);
K14 = R(3,2,:) - R(2,3,:);

K22 = R(2,2,:) - R(1,1,:) - R(3,3,:);
K23 = R(2,3,:) + R(3,2,:);
K24 = R(1,3,:) - R(3,1,:);

K33 = R(3,3,:) - R(1,1,:) - R(2,2,:);
K34 = R(2,1,:) - R(1,2,:);

K44 = R(1,1,:) + R(2,2,:) + R(3,3,:);

% Construct K matrix according to paper
K = [...
    K11,    K12,    K13,    K14;
    K12,    K22,    K23,    K24;
    K13,    K23,    K33,    K34;
    K14,    K24,    K34,    K44];

K = K ./ 3;

% For each input rotation matrix, calculate the corresponding eigenvalues
% and eigenvectors. The eigenvector corresponding to the largest eigenvalue
% is the unit quaternion representing the same rotation.
for i = 1:size(R,3)
    [eigVec,eigVal] = eig(K(:,:,i),'vector');
    [~,maxIdx] = max(real(eigVal));
    quat(i,:) = real([eigVec(4,maxIdx) eigVec(1,maxIdx) eigVec(2,maxIdx) eigVec(3,maxIdx)]);
    
    % By convention, always keep scalar quaternion element positive. 
    % Note that this does not change the rotation that is represented
    % by the unit quaternion, since q and -q denote the same rotation.
    if quat(i,1) < 0
        quat(i,:) = -quat(i,:);
    end
end

end