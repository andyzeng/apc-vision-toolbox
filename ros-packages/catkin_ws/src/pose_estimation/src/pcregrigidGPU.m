function [tform, movingReg, rmse] = pcregrigidGPU(moving, fixed, varargin)
% GPU version of Matlab's pcregrigid
% pcregrigid Register two point clouds with ICP algorithm.
%   tform = pcregrigid(moving, fixed) returns the rigid transformation
%   that registers the moving point cloud with the fixed point cloud. moving and
%   fixed are pointCloud object. tform is an affine3d object that describes the rigid 
%   3-D transform. The rigid transformation between the moving
%   and fixed are estimated by Iterative Closest Point (ICP) algorithm. 
%
%   [tform, movingReg] = pcregrigid(moving, fixed) additionally
%   returns the transformed  point cloud, movingReg, that is aligned with
%   the fixed point cloud.
%
%   [..., rmse] = pcregrigid(moving, fixed) additionally returns the
%   root mean squared error of the Euclidean distance between the aligned
%   point clouds.
% 
%   [...] = pcregrigid(...,Name, Value) specifies additional
%   name-value pairs described below:
%
%   'Metric'            A string used to specify the metric of the
%                       minimization function. The ICP algorithm minimized
%                       the distance between the two point clouds according
%                       to the given metric. Valid strings are
%                       'pointToPoint' and 'pointToPlane'. Setting Metric
%                       to 'pointToPlane' can reduce the number of
%                       iterations to process. However, this metric
%                       requires extra algorithmic steps within each
%                       iteration. The 'pointToPlane' metric helps
%                       registration of planar surfaces.
%
%                       Default: 'pointToPoint'
%
%   'Extrapolate'       A boolean to turn on/off the extrapolation step
%                       that traces out a path in the registration state
%                       space, described in the original paper of ICP by
%                       Besl and McKay (1992). This may reduce the number
%                       of iterations to converge.
%                           
%                       Default: false
%
%   'InlierRatio'       A scalar to specify the percentage of inliers.
%                       During an ICP iteration, every point in the moving
%                       point cloud is matched to its nearest neighbor in
%                       the fixed point cloud. The pair of matched points
%                       is considered as an inlier if its Euclidean
%                       distance falls into the given percentage of the
%                       distribution of matching distance. By default, all
%                       matching pairs are used.
%
%                       Default: 1
%
%   'MaxIterations'     A positive integer to specify the maximum number 
%                       of iterations before ICP stops.
%
%                       Default: 20
%
%   'Tolerance'         A 2-element vector, [Tdiff, Rdiff], to specify
%                       the tolerance of absolute difference in translation
%                       and rotation estimated in consecutive ICP
%                       iterations. Tdiff measures the Euclidean distance
%                       between two translation vectors, while Rdiff
%                       measures the angular difference in radians. The
%                       algorithm stops when the average difference between
%                       estimated rigid transformations in the three most
%                       recent consecutive iterations falls below the
%                       specified tolerance value.
%
%                       Default: [0.01, 0.009] 
%
%   'InitialTransform'  An affine3d object to specify the initial rigid
%                       transformation. This is useful when a coarse
%                       estimation can be provided externally.
%
%                       Default: affine3d()
%
%   'Verbose'           Set true to display progress information.
%
%                       Default: false
%
%   Notes
%   -----
%   The registration algorithm is based on "Iterative Closest Point" (ICP)
%   algorithm, which is an iterative process. Best performance might
%   require adjusting the options for different data.
%
%   Class Support 
%   ------------- 
%   moving and fixed must be pointCloud object.
%
%   Example: Align two point clouds
%   --------------------------------
%   % load data
%   ptCloud = pcread('teapot.ply');
%   figure
%   showPointCloud(ptCloud); 
%   title('Teapot');
%
%   % Create a transform object with 30 degree rotation along z-axis and
%   % translation [5, 5, 10]
%   A = [cos(pi/6) sin(pi/6) 0 0; ...
%       -sin(pi/6) cos(pi/6) 0 0; ...
%               0         0  1 0; ...
%               5         5 10 1];
%   tform1 = affine3d(A);
%
%   % Transform the point cloud
%   ptCloudTformed = pctransform(ptCloud, tform1);
%
%   figure
%   showPointCloud(ptCloudTformed);
%   title('Transformed Teapot');
%
%   % Apply the rigid registration
%   tform = pcregrigid(ptCloudTformed, ptCloud, 'Extrapolate', true);
%
%   % Compare the result with the true transformation
%   disp(tform1.T);
%   tform2 = invert(tform);
%   disp(tform2.T);
%
% See also pointCloud, pctransform, affine3d, showPointCloud, pcdownsample, pcdenoise, pcmerge
 
% Copyright 2014 The MathWorks, Inc.
%
% References
% ----------
% Besl, Paul J.; N.D. McKay (1992). "A Method for Registration of 3-D
% Shapes". IEEE Trans. on Pattern Analysis and Machine Intelligence (Los
% Alamitos, CA, USA: IEEE Computer Society) 14 (2): 239�256.
%
% Chen, Yang; Gerard Medioni (1991). "Object modelling by registration of
% multiple range images". Image Vision Comput. (Newton, MA, USA:
% Butterworth-Heinemann): 145�155

global KNNSearchGPU

narginchk(2, 16);

% Validate inputs
[metric, doExtrapolate, inlierRatio, maxIterations, tolerance, ...
    initialTransform, verbose] = validateAndParseOptInputs(moving, fixed, varargin{:});
   
printer = vision.internal.MessagePrinter.configure(verbose);

% A copy of the input with unorganized M-by-3 data
ptCloudA = removeInvalidPoints(moving);
ptCloudB = removeInvalidPoints(fixed);

% At least three points are needed to determine a 3-D transformation
if ptCloudA.Count < 3 || ptCloudB.Count < 3
    error(message('vision:pointcloud:notEnoughPoints'));
end

% Normal vector is needed for PointToPlane metric
if strcmpi(metric, 'PointToPlane')
    normalB = ptCloudB.Normal;
    % Compute the unit normal vector if it is not provided.
    if isempty(normalB)
        numNeighborPoints = 5;
        normalB = vision.internal.pointCloudNormal(ptCloudB.Location, numNeighborPoints);
    end
end

Rs = zeros(3, 3, maxIterations+1);
Ts = zeros(3, maxIterations+1);
% Quaternion and translation vector
qs = [ones(1, maxIterations+1); zeros(6, maxIterations+1)];
% The difference of quaternion and translation vector in consecutive
% iterations
dq = zeros(7, maxIterations+1);
% The angle between quaternion and translation vectors in consecutive
% iterations
dTheta = zeros(maxIterations+1, 1);
% RMSE
Err = zeros(maxIterations+1, 1); 

% Apply the initial condition.
% We use pre-multiplication format in this algorithm.
Rs(:,:,1) = initialTransform.T(1:3, 1:3)';
Ts(:,1) = initialTransform.T(4, 1:3)';
qs(:,1) = [vision.internal.quaternion.rotationToQuaternion(Rs(:,:,1)); Ts(:,1)];

locA = ptCloudA.Location;
if qs(1) ~= 0 || any(qs(2:end,1))
    locA = rigidTransform(ptCloudA.Location, Rs(:,:,1), Ts(:,1));
end

stopIteration = maxIterations;
upperBound = max(1, round(inlierRatio(1)*ptCloudA.Count));    

% Start ICP iterations
for i = 1 : maxIterations
    printer.linebreak;
    printer.print('--------------------------------------------\n');
    printer.printMessage('vision:pointcloud:icpIteration',i);
    printer.printMessageNoReturn('vision:pointcloud:findCorrespondenceStart');
    
    % Find the correspondence
    [indices, dists] = multiQueryKNNSearchImplGPU(ptCloudB, locA);
%     numBlocks = ceil(size(locA,1)/512);
%     numThreads = 512;
%     KNNSearchGPU.GridSize = [numBlocks 1];
%     KNNSearchGPU.ThreadBlockSize = [numThreads 1];
%     resultsGPU = gpuArray(single(zeros(1,2*size(locA,1))));
%     argsGPU = gpuArray(int32([numBlocks numThreads size(ptCloudB.Location,1) size(locA,1)]));
%     pcGPU1 = gpuArray(single([ptCloudB.Location(:,1)' ptCloudB.Location(:,2)' ptCloudB.Location(:,3)']));
%     pcGPU2 = gpuArray(single([locA(:,1)' locA(:,2)' locA(:,3)']));
%     results = feval(KNNSearchGPU,resultsGPU,argsGPU,pcGPU1,pcGPU2);
%     indices = gather(results(1:size(locA,1)));
%     dists = gather(results((size(locA,1)+1):end));

    % Remove outliers
    keepInlierA = false(ptCloudA.Count, 1); 
    [~, idx] = sort(dists);
    keepInlierA(idx(1:upperBound)) = true;
    inlierIndicesA = find(keepInlierA);
    inlierIndicesB = indices(keepInlierA);
    inlierDist = dists(keepInlierA);
    
    if numel(inlierIndicesA) < 3
        error(message('vision:pointcloud:notEnoughPoints'));
    end

    printer.printMessage('vision:pointcloud:stepCompleted');

    if i == 1
        Err(i) = sqrt(sum(inlierDist)/length(inlierDist));
    end
    
    printer.printMessageNoReturn('vision:pointcloud:estimateTransformStart');
    
    % Estimate transformation given correspondences
    if strcmpi(metric, 'PointToPoint')
        [R, T] = minimizePointToPointMetric(locA(inlierIndicesA, :), ...
                    ptCloudB.Location(inlierIndicesB, :));
    else % PointToPlane
        [R, T] = minimizePointToPlaneMetric(locA(inlierIndicesA, :), ...
                    ptCloudB.Location(inlierIndicesB, :), normalB(inlierIndicesB, :));
    end        

    % Bad correspondence may lead to singular matrix
    if any(isnan(T))||any(isnan(R(:)))
        error(message('vision:pointcloud:singularMatrix'));
    end
    
    % Update the total transformation
    Rs(:,:,i+1) = R * Rs(:,:,i);
    Ts(:,i+1) = R * Ts(:,i) + T;
    
    printer.printMessage('vision:pointcloud:stepCompleted');

    % RMSE
    locA = rigidTransform(ptCloudA.Location, Rs(:,:,i+1), Ts(:,i+1));
    squaredError = sum((locA(inlierIndicesA, :) - ptCloudB.Location(inlierIndicesB, :)).^2, 2);
    Err(i+1) = sqrt(sum(squaredError)/length(squaredError));
    
    % Convert to vector representation
    qs(:,i+1) = [vision.internal.quaternion.rotationToQuaternion(Rs(:,:,i+1)); Ts(:,i+1)];
    
    % With extrapolation, we might be able to converge faster
    if doExtrapolate
        printer.printMessageNoReturn('vision:pointcloud:updateTransformStart');

        extrapolateInTransformSpace;

        printer.printMessage('vision:pointcloud:stepCompleted');
    end
    
    % Check convergence    
    % Compute the mean difference in R/T from the recent three iterations.
    [dR, dT] = getChangesInTransformation;
    
    printer.printMessage('vision:pointcloud:checkConverge',num2str(tdiff), num2str(rdiff), num2str(Err(i+1)));

    % Stop ICP if it already converges
    if dT <= tolerance(1) && dR <= tolerance(2)
        stopIteration = i;
        break;
    end
end

% Make the R to be orthogonal as much as possible
R = Rs(:,:,stopIteration+1)';
[U, ~, V] = svd(R);
R = U * V';
tformMatrix = [R, zeros(3,1);...
               Ts(:, stopIteration+1)',  1];
tform = affine3d(tformMatrix);
rmse = Err(stopIteration+1);

printer.linebreak;
printer.print('--------------------------------------------\n');
printer.printMessage('vision:pointcloud:icpSummary',stopIteration, num2str(rmse));

if nargout >= 2
    movingReg = pctransform(moving, tform);
end

    %======================================================================
    % Nested function to perform extrapolation
    % Besl, P., & McKay, N. (1992). A method for registration of 3-D shapes. 
    % IEEE Transactions on pattern analysis and machine intelligence, p245.
    %======================================================================
    function extrapolateInTransformSpace        
        dq(:,i+1) = qs(:,i+1) - qs(:,i);
        n1 = norm(dq(:,i));
        n2 = norm(dq(:,i+1));
        dTheta(i+1) = (180/pi)*acos(dot(dq(:,i),dq(:,i+1))/(n1*n2));

        angleThreshold = 10;
        scaleFactor = 25;
        if i > 2 && dTheta(i+1) < angleThreshold && dTheta(i) < angleThreshold
            d = [Err(i+1), Err(i), Err(i-1)];
            v = [0, -n2, -n1-n2];
            vmax = scaleFactor * n2;
            dv = extrapolate(v,d,vmax);
            if dv ~= 0
                q = qs(:,i+1) + dv * dq(:,i+1)/n2;
                q(1:4) = q(1:4)/norm(q(1:4));
                % Update transformation and data
                qs(:,i+1) = q;
                Rs(:,:,i+1) = vision.internal.quaternion.quaternionToRotation(q(1:4));
                Ts(:,i+1) = q(5:7);
                locA = rigidTransform(ptCloudA.Location, Rs(:,:,i+1), Ts(:,i+1));
            end
        end
    end

    %======================================================================
    % Nested function to compute the changes in rotation and translation
    %======================================================================
    function [dR, dT] = getChangesInTransformation
        dR = 0;
        dT = 0;
        count = 0;
        for k = max(i-2,1):i
            % Rotation difference in radians
            rdiff = acos(dot(qs(1:4,k),qs(1:4,k+1))/(norm(qs(1:4,k))*norm(qs(1:4,k+1))));
            % Euclidean difference
            tdiff = sqrt(sum((Ts(:,k)-Ts(:,k+1)).^2));
            dR = dR + rdiff;
            dT = dT + tdiff;
            count = count + 1;
        end
        dT = dT/count;
        dR = dR/count;
    end

end

%==========================================================================
% Parameter validation
%==========================================================================
function [metric, doExtrapolate, inlierRatio, maxIterations, tolerance, ...
            initialTransform, verbose] = validateAndParseOptInputs(moving, fixed, varargin)

    validateattributes(moving, {'pointCloud'},{},mfilename, 'moving');
    validateattributes(fixed, {'pointCloud'},{},mfilename, 'fixed');
            
    % Parse the input P-V pairs
    defaults = struct(...
        'Metric', 'PointToPoint', ...
        'Extrapolate',  false, ...
        'InlierRatio', 1.0,...
        'MaxIterations', 20,...
        'Tolerance', [0.01, 0.009],...
        'InitialTransform', affine3d(),...
        'Verbose', false);

    parser = inputParser;
    parser.CaseSensitive = false;

    parser.addParameter('Metric', defaults.Metric);    
    parser.addParameter('Extrapolate', defaults.Extrapolate, ...
                @(x)validateattributes(x,{'logical'}, {'scalar','nonempty'}));
    parser.addParameter('InlierRatio', defaults.InlierRatio, ...
                @(x)validateattributes(x,{'single', 'double'}, {'real','nonempty','scalar','>',0,'<=',1}));
    parser.addParameter('MaxIterations', defaults.MaxIterations, ...
                @(x)validateattributes(x,{'single', 'double'}, {'scalar','integer'}));
    parser.addParameter('Tolerance', defaults.Tolerance, ...
                @(x)validateattributes(x,{'single', 'double'}, {'real','nonnegative','numel', 2}));        
    parser.addParameter('InitialTransform', defaults.InitialTransform, ...
                @(x)validateattributes(x,{'affine3d'}, {'scalar'}));
    parser.addParameter('Verbose', defaults.Verbose, ...
                @(x)validateattributes(x,{'logical'}, {'scalar','nonempty'}));        
            
    parser.parse(varargin{:});

    metric          = validatestring(parser.Results.Metric, {'PointToPoint', 'PointToPlane'}, mfilename, 'Metric');
    doExtrapolate   = parser.Results.Extrapolate;
    inlierRatio     = parser.Results.InlierRatio;
    maxIterations   = parser.Results.MaxIterations;
    tolerance       = parser.Results.Tolerance;
    initialTransform = parser.Results.InitialTransform;
    if ~(isRigidTransform(initialTransform))
        error(message('vision:pointcloud:rigidTransformOnly'));
    end
    verbose = parser.Results.Verbose;
end

%==========================================================================
% Determine if transformation is rigid transformation
%==========================================================================
function tf = isRigidTransform(tform)

singularValues = svd(tform.T(1:tform.Dimensionality,1:tform.Dimensionality));
tf = max(singularValues)-min(singularValues) < 100*eps(max(singularValues(:)));
tf = tf && abs(det(tform.T)-1) < 100*eps(class(tform.T));

end
        
%==========================================================================
function B = rigidTransform(A, R, T)
    B = A * R';
    B(:,1) = B(:,1) + T(1);
    B(:,2) = B(:,2) + T(2);
    B(:,3) = B(:,3) + T(3);
end

%==========================================================================
% Solve the following minimization problem:
%       min_{R, T} sum(|R*p+T-q|^2)
%
% p, q are all N-by-3 matrix
%
% The problem is solved by SVD
%==========================================================================
function [R, T] = minimizePointToPointMetric(p, q)
    n = size(p, 1);
    m = size(q, 1);

    % Find data centroid and deviations from centroid
    pmean = sum(p,1)/n;
    p2 = p - repmat(pmean, n, 1);

    qmean = sum(q,1)/m;
    q2 = q - repmat(qmean, m, 1);

    % Covariance matrix
    C = p2'*q2; 

    [U,~,V] = svd(C); 

    % Handle the reflection case
    R = V*diag([1 1 sign(det(U*V'))])*U';

    % Compute the translation
    T = qmean' - R*pmean';
end

%==========================================================================
% Solve the following minimization problem:
%       min_{R, T} sum(|dot(R*p+T-q,nv)|^2)
%
% p, q, nv are all N-by-3 matrix, and nv is the unit normal at q
%
% Here the problem is solved by linear approximation to the rotation matrix
% when the angle is small.
%==========================================================================
function [R, T] = minimizePointToPlaneMetric(p, q, nv)
    % Set up the linear system
    cn = [cross(p,nv,2),nv];
    C = cn'*cn;
    qp = q-p;
    b =  [sum(sum(qp.*repmat(cn(:,1),1,3).*nv, 2));
          sum(sum(qp.*repmat(cn(:,2),1,3).*nv, 2));
          sum(sum(qp.*repmat(cn(:,3),1,3).*nv, 2));
          sum(sum(qp.*repmat(cn(:,4),1,3).*nv, 2));
          sum(sum(qp.*repmat(cn(:,5),1,3).*nv, 2));
          sum(sum(qp.*repmat(cn(:,6),1,3).*nv, 2))];
    % X is [alpha, beta, gamma, Tx, Ty, Tz]
    X = C\b;

    cx = cos(X(1)); 
    cy = cos(X(2)); 
    cz = cos(X(3)); 
    sx = sin(X(1)); 
    sy = sin(X(2)); 
    sz = sin(X(3)); 

    R = [cy*cz, sx*sy*cz-cx*sz, cx*sy*cz+sx*sz;
         cy*sz, cx*cz+sx*sy*sz, cx*sy*sz-sx*cz;
           -sy,          sx*cy,          cx*cy];

    T = X(4:6);
end

%==========================================================================
% Extrapolation in quaternion space. Details are found in:
% Besl, P., & McKay, N. (1992). A method for registration of 3-D shapes. 
% IEEE Transactions on pattern analysis and machine intelligence, 239-256.
%==========================================================================
function dv = extrapolate(v,d,vmax)
    p1 = polyfit(v,d,1);    % linear fit
    p2 = polyfit(v,d,2);    % parabolic fit
    v1 = -p1(2)/p1(1);      % linear zero crossing point
    v2 = -p2(2)/(2*p2(1));  % polynomial top point

    if (issorted([0 v2 v1 vmax]) || issorted([0 v2 vmax v1]))
        % Parabolic update
        dv = v2;
    elseif (issorted([0 v1 v2 vmax]) || issorted([0 v1 vmax v2])...
            || (v2 < 0 && issorted([0 v1 vmax])))
        % Line update
        dv = v1;
    elseif (v1 > vmax && v2 > vmax)
        % Maximum update
        dv = vmax;
    else
        % No extrapolation
        dv = 0;
    end
end