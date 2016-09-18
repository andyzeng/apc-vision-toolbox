% User configurations (change me)
toolboxPath = '../../../../..'; % Directory of toolbox utilities
tmpDataPath = fullfile(toolboxPath,'data/tmp'); % Temporary directory used by marvin_convnet, where all RGB-D images and detection masks are saved
modelsPath = './models/objects'; % Directory holding pre-scanned object models
scenePath = fullfile(toolboxPath,'data/sample/scene-0000'); % Directory holding the RGB-D data of the sample scene
calibPath = fullfile(toolboxPath,'data/sample/calibration'); % Directory holding camera pose calibration data for the sample scene

% Add paths and create directories
addpath(genpath(fullfile(toolboxPath,'rgbd-utils')));
addpath(genpath(fullfile(toolboxPath,'vis-utils')));

% Get set of colors for visualizations (from vis-utils)
load('colorPalette.mat');

% Load sample scene data
fprintf('    [Processing] Loading scene RGB-D data\n');
sceneData = loadScene(tmpDataPath);
numFrames = length(sceneData.colorFrames);

% Calibrate scene
sceneData = loadCalib(calibPath,sceneData);

% Load pre-scanned object models
global objNames;
global objModels;
objNames = {'barkely_hide_bones', ...
            'cherokee_easy_tee_shirt', ...
            'clorox_utility_brush', ...
            'cloud_b_plush_bear', ...
            'cool_shot_glue_sticks', ...
            'command_hooks', ...
            'crayola_24_ct', ...
            'creativity_chenille_stems', ...
            'dasani_water_bottle', ...
            'dove_beauty_bar', ...
            'dr_browns_bottle_brush', ...
            'easter_turtle_sippy_cup', ...
            'elmers_washable_no_run_school_glue', ...
            'expo_dry_erase_board_eraser', ...
            'fiskars_scissors_red', ...
            'fitness_gear_3lb_dumbbell', ...
            'folgers_classic_roast_coffee', ...
            'hanes_tube_socks', ...
            'i_am_a_bunny_book', ...
            'jane_eyre_dvd', ...
            'kleenex_paper_towels', ...
            'kleenex_tissue_box', ...
            'kyjen_squeakin_eggs_plush_puppies', ...
            'laugh_out_loud_joke_book', ...
            'oral_b_toothbrush_green', ...
            'oral_b_toothbrush_red', ...
            'peva_shower_curtain_liner', ...
            'platinum_pets_dog_bowl', ...
            'rawlings_baseball', ...
            'rolodex_jumbo_pencil_cup', ...
            'safety_first_outlet_plugs', ...
            'scotch_bubble_mailer', ...
            'scotch_duct_tape', ...
            'soft_white_lightbulb', ...
            'staples_index_cards', ...
            'ticonderoga_12_pencils', ...
            'up_glucose_bottle', ...
            'woods_extension_cord', ...
            'womens_knit_gloves'};
objModels = cell(1,length(objNames));
fprintf('    [Processing] Loading pre-scanned object models\n');
for objIdx = 1:length(objNames)
  try
    objModels{objIdx} = pcread(sprintf('models/objects/%s.ply',objNames{objIdx}));
  end
end

% Load pre-scanned empty tote and shelf bins
global emptyShelfModels;
global emptyToteModel;
emptyShelfModels = cell(1,12);
binIds = 'ABCDEFGHIJKL';
fprintf('    [Processing] Loading pre-scanned empty tote and shelf bins\n');
for binIdx = 1:length(binIds)
    emptyShelfModels{binIdx} = pcread(sprintf('models/bins/bin%s.ply',binIds(binIdx)));
end
emptyToteModel = pcread('models/bins/tote.ply');

global visPath; visPath = fullfile(pwd,'visualizations');
global savePointCloudVis; savePointCloudVis = false;
global saveResultImageVis; saveResultImageVis = false;
global useGPU; useGPU = false;

% Fill holes in depth frames for scene
for frameIdx = 1:length(sceneData.depthFrames)
    sceneData.depthFrames{frameIdx} = fillHoles(sceneData.depthFrames{frameIdx});
end

% Load scene point cloud
fprintf('    [Processing] Loading scene point clouds\n');
scenePointCloud = getScenePointCloud(sceneData);

% Load and align empty bin/tote point cloud to observation
binIds = 'ABCDEFGHIJKL';
if strcmp(sceneData.env,'shelf')
    backgroundPointCloud = emptyShelfModels{strfind(binIds,sceneData.binId)};
else
    backgroundPointCloud = emptyToteModel;
end
[tform,backgroundPointCloud] = pcregrigid(backgroundPointCloud,scenePointCloud,'InlierRatio',0.8,'MaxIterations',200,'Tolerance',[0.0001,0.0009],'Verbose',false,'Extrapolate',true);
extBin2Bg = inv(tform.T');

if savePointCloudVis
  pcwrite(scenePointCloud,fullfile(visPath,'vis.pc.scene'),'PLYFormat','binary');
  pcwrite(backgroundPointCloud,fullfile(visPath,'vis.pc.bg'),'PLYFormat','binary');
end

% Parse object names
objList = sceneData.objects;
uniqueObjectList = unique(objList);
for objIdx = 1:length(uniqueObjectList)
    objName = uniqueObjectList{objIdx};
    objNum = sum(ismember(objList,objName));
    objModel = objModels{find(ismember(objNames,objName))};
    if isempty(objModel)
        fprintf('Error: object model not loaded for %s!\n',objName);
    end

    % Do 6D pose estimation for each object and save the results
    objPoses = getObjectPose(tmpDataPath,sceneData,scenePointCloud,backgroundPointCloud,extBin2Bg,objName,objModel,objNum);
end

fprintf('    [Visualization] Drawing predicted object poses\n');
% Load results (predicted 6D object poses)
resultFiles = dir(fullfile(tmpDataPath,'results/*.result.txt'));
results = cell(1,length(resultFiles));
confScores = [];
for resultIdx = 1:length(resultFiles)
    tmpResultFile = resultFiles(resultIdx).name;
    tmpResultFilenameDotIdx = strfind(tmpResultFile,'.');
    tmpResult.objName = tmpResultFile(1:(tmpResultFilenameDotIdx(1)-1));
    tmpResult.objNum = str2double(tmpResultFile((tmpResultFilenameDotIdx(1)+1):(tmpResultFilenameDotIdx(2)-1)));
    tmpResult.objPoseWorld = eye(4);
    tmpResult.objPoseWorld(1:3,4) = dlmread(fullfile(tmpDataPath,'results',tmpResultFile),'\t',[1,0,1,2])';
    objPoseRotQuat = dlmread(fullfile(tmpDataPath,'results',tmpResultFile),'\t',[4,0,4,3]);
    tmpResult.objPoseWorld(1:3,1:3) = quat2rotm([objPoseRotQuat(4),objPoseRotQuat(1:3)]);
    tmpResult.confScore = dlmread(fullfile(tmpDataPath,'results',tmpResultFile),'\t',[28,0,28,0]);
    confScores = [confScores;tmpResult.confScore];
    results{resultIdx} = tmpResult;
end

% Sort results by confidence scores
[~,sortIdx] = sortrows(confScores,1);

% Get random colors per object in the scene
randColorIdx = randperm(length(colorPalette),length(results));

% Create canvas for visualization
canvas = sceneData.colorFrames{8};
canvasSeg = uint8(ones(size(canvas))*255);
canvasPose = canvas;
canvasPose = insertText(canvasPose,[10 10],'Confidence  :  Object','Font','LucidaSansDemiBold','FontSize',12,'TextColor','white','BoxColor','black');

% Loop through each object to save and visualize predicted object pose
uniqueObjList = unique(sceneData.objects);
textPosY = 32;
for resultIdx = 1:length(resultFiles)
    currResult = results{sortIdx(resultIdx)};
    objColor = colorPalette{randColorIdx(resultIdx)};

    % Load pre-scanned object point cloud
    objPointCloud = pcread(fullfile(modelsPath,sprintf('%s.ply',currResult.objName)));

    % Draw projected object model and bounding box
    [canvasSeg,canvasPose] = showObjectPose(currResult.objName, canvasSeg, canvasPose, sceneData.colorFrames{8}, sceneData.depthFrames{8}, sceneData.extCam2World{8}, sceneData.colorK, objPointCloud, currResult.objPoseWorld, objColor);                             
    canvasPose = insertText(canvasPose,[10 textPosY],sprintf('  %f  :  %s',currResult.confScore,currResult.objName),'Font','LucidaSansDemiBold','FontSize',12,'TextColor',objColor,'BoxColor','black');
    textPosY = textPosY + 22;
end

imshow(canvasPose); title('Predicted 6D Object Poses');













