function annotator()

close all;

global scene;
scene = pcread('~/apc/toolbox/data/pointclouds/scenes/warehouse_competition_pick_seq-0000.ply');


global model;
model = pcread('~/apc/toolbox/data/pointclouds/objects/expo_dry_erase_board_eraser.ply');

global resultFile;
resultFile = 'test.txt';

global tform;
try
    tform = dlmread(resultFile);
catch
    tform = eye(4);
    tform(1,4) = mean(scene.XLimits);
    tform(2,4) = mean(scene.YLimits);
    tform(3,4) = mean(scene.ZLimits);
end


dispModel = pctransform(model,affine3d(tform'));
figure(1); pcshow(pcmerge(scene,dispModel,0.001));






screenSize = get(0,'Screensize');
set(gcf,'Position',screenSize); % maximize figure


sldTX = uicontrol('Style','slider',...
                  'Min',scene.XLimits(1),'Max',scene.XLimits(2),'Value',mean(scene.XLimits),...
                  'Position', [50,135,screenSize(3)-200,20],...
                  'Callback',@uTX); 
sldTY = uicontrol('Style','slider',...
                  'Min',scene.YLimits(1),'Max',scene.YLimits(2),'Value',mean(scene.YLimits),...
                  'Position', [50,110,screenSize(3)-200,20],...
                  'Callback',@uTY); 
sldTZ = uicontrol('Style','slider',...
                  'Min',scene.ZLimits(1),'Max',scene.ZLimits(2),'Value',mean(scene.ZLimits),...
                  'Position', [50,85,screenSize(3)-200,20],...
                  'Callback',@uTZ); 
              
sldRX = uicontrol('Style','slider',...
                  'Min',-pi,'Max',pi,'Value',0,...
                  'Position', [50,60,screenSize(3)-200,20],...
                  'Callback',@uRX); 
              
sldRY = uicontrol('Style','slider',...
                  'Min',-pi,'Max',pi,'Value',0,...
                  'Position', [50,35,screenSize(3)-200,20],...
                  'Callback',@uRY); 
              
sldRZ = uicontrol('Style','slider',...
                  'Min',-pi,'Max',pi,'Value',0,...
                  'Position', [50,10,screenSize(3)-200,20],...
                  'Callback',@uRZ); 

end

function uPC(tform)
global scene;
global model;
dispModel = pctransform(model,affine3d(tform'));
figure(1);
currCamPos = campos;
pcshow(pcmerge(scene,dispModel,0.001));
campos(currCamPos)
global resultFile;
dlmwrite(resultFIle
end

function uTX(source,callbackdata)
global tform;
tform(1,4) = source.Value;
uPC(tform);
end

function uTY(source,callbackdata)
global tform;
tform(2,4) = source.Value;
uPC(tform);
end

function uTZ(source,callbackdata)
global tform;
tform(3,4) = source.Value;
uPC(tform);
end

function uRX(source,callbackdata)
global tform;
tform(1:3,1:3) = vrrotvec2mat([1,0,0,source.Value]);
uPC(tform);
end

function uRY(source,callbackdata)
global tform;
tform(1:3,1:3) = vrrotvec2mat([0,1,0,source.Value]);
uPC(tform);
end

function uRZ(source,callbackdata)
global tform;
tform(1:3,1:3) = vrrotvec2mat([0,0,1,source.Value]);
uPC(tform);
end









