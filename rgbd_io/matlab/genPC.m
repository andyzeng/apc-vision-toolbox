function [seqIm,seqCloud] = genPC(seqData)

    % Create aggregated RGB-D point cloud
    gridFilterKernelSize = 0.001;
    numFrames = length(seqData.colorFrames);
    for frameIdx = 1:numFrames
        color = seqData.colorFrames{frameIdx};
        depth = seqData.depthFrames{frameIdx};
        extCam2World = seqData.extCam2World{frameIdx};
        extCam2Bin = inv(seqData.extBin2World)*extCam2World;

        % Project depth into camera space
        [pixX,pixY] = meshgrid(1:640,1:480);
        camX = (pixX-seqData.colorK(1,3)).*depth/seqData.colorK(1,1);
        camY = (pixY-seqData.colorK(2,3)).*depth/seqData.colorK(2,2);
        camZ = depth;

        % Only use points with valid depth
        validDepth = find((camZ > 0));
        camPts = [camX(validDepth),camY(validDepth),camZ(validDepth)]';
        camPts = extCam2Bin(1:3,1:3) * camPts + repmat(extCam2Bin(1:3,4),1,size(camPts,2));

        % Get vertex colors
        colorR = color(:,:,1);
        colorG = color(:,:,2);
        colorB = color(:,:,3);
        colorPts = [colorR(validDepth),colorG(validDepth),colorB(validDepth)]';

        % Remove out of bounds points
        if strcmp(seqData.env,'tote')
          viewBounds = [-0.3, 0.3; -0.4, 0.4; -0.05, 0.2];
        else
          viewBounds = [-0.01, 0.40; -0.17, 0.17; -0.06, 0.20];
        end
        ptsOutsideBounds = find((camPts(1,:) < viewBounds(1,1)) | (camPts(1,:) > viewBounds(1,2)) | ...
                                (camPts(2,:) < viewBounds(2,1)) | (camPts(2,:) > viewBounds(2,2)) | ...
                                (camPts(3,:) < viewBounds(3,1)) | (camPts(3,:) > viewBounds(3,2)));
        camPts(:,ptsOutsideBounds) = [];
        colorPts(:,ptsOutsideBounds) = [];
        
        % Transform points to world space
        camPts = seqData.extBin2World(1:3,1:3) * camPts + repmat(seqData.extBin2World(1:3,4),1,size(camPts,2));
        
        % Aggregate point clouds
        if frameIdx == 1
          camCloud = pointCloud(camPts','Color',colorPts');
        else
          camCloud = pcmerge(camCloud,pointCloud(camPts','Color',colorPts'),gridFilterKernelSize);
        end
    end
        
    % Save representative image
    if strcmp(seqData.env,'shelf')
        seqIm = seqData.colorFrames{8};
    else
        im1 = seqData.colorFrames{5};
        im2 = seqData.colorFrames{14};
        seqIm = uint8(zeros(960,1280,3));
        seqIm(1:960,321:960,:) = [im1;im2];
    end

    % Downsample point cloud
    seqCloud = pcdownsample(camCloud,'gridAverage',gridFilterKernelSize);

end

