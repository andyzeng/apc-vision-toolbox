clear all; close all;

% Parameters
dataPath = '/home/andyzeng/apc/toolbox/data/benchmark/warehouse/practice/shelf';
calibDir = '/home/andyzeng/apc/toolbox/data/benchmark/warehouse/calibration';
dataPrefix = 'wps';

% Load webpage templates
pagePath = '~/apc/rgbd-annotator';
fid = fopen(fullfile(pagePath,'index.html'),'rt') ;
pageIndex = fread(fid) ;
fclose(fid) ;
pageIndex = char(pageIndex.') ;
fid = fopen(fullfile(pagePath,'src','index.js'),'rt') ;
pageJS = fread(fid) ;
fclose(fid) ;
pageJS = char(pageJS.') ;

pcPath = '/home/andyzeng/apc/rgbd-annotator/data/pointclouds/scenes';
imPath = '/home/andyzeng/apc/rgbd-annotator/data/images/scenes';
addpath(genpath('../../calibration'));
dataDir = dir(fullfile(dataPath,'seq-*'));
for seqIdx = 1:length(dataDir)
    seqName = dataDir(seqIdx).name;
    seqPath = fullfile(dataPath,seqName);
    
    % Skip images
    if ~isempty(strfind(seqName,'.png'))
        continue;
    end

    % Load RGB-D sequence
    fprintf('%s\n',seqPath);
    seqData = loadSeq(seqPath);
    
    % Apply calibrated camera poses
    seqData = loadCalib(calibDir,seqData);
    
    % Create representative image and point cloud
    [seqIm,seqPC] = genPC(seqData);
    imwrite(seqIm,fullfile(imPath,strcat(dataPrefix,seqName(5:end),'.jpg')));
    pcwrite(seqPC,fullfile(pcPath,strcat(dataPrefix,seqName(5:end))),'PLYFormat','binary');
    
    % Create webpage for each scene/object pair
    sceneName = strcat(dataPrefix,seqName(5:end));
    objList = unique(seqData.objects);
    for objIdx = 1:length(objList)
        objName = objList{objIdx};
        numInst = sum(ismember(seqData.objects,objName));
        for objInst = 1:numInst
            pageName = strcat(sceneName,'_',objName,'_',num2str(objInst));
            
            % Create html page
            tmpPageIndex = strrep(pageIndex,'SCENE_PLACEHOLDER',sceneName);
            tmpPageIndex = strrep(tmpPageIndex,'OBJECT_PLACEHOLDER',objName);
            tmpPageIndex = strrep(tmpPageIndex,'LINK_PLACEHOLDER',pageName);
            fid = fopen(fullfile(pagePath,strcat(pageName,'.html')),'wt');
            fwrite(fid,tmpPageIndex);
            fclose (fid);
            
            % Create js file
            tmpPageJS = strrep(pageJS,'SCENE_PLACEHOLDER',sceneName);
            tmpPageJS = strrep(tmpPageJS,'OBJECT_PLACEHOLDER',objName);
            fid = fopen(fullfile(pagePath,'src',strcat(pageName,'.js')),'wt');
            fwrite(fid,tmpPageJS);
            fclose(fid);
            
        end
    end
end


