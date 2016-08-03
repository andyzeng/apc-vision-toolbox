clear all; close all;

toolboxRoot = '../..';
addpath(genpath(fullfile(toolboxRoot,'rgbd_io','matlab')));

% converOld2NewFormat('../../sample_data/test/000000','../../sample_data/test/seq-000000')



inputFolder = '../../sample_data/test/000000';
outputFolder = '../../sample_data/test/seq-000000';

% Make output folder
mkdir(outputFolder);

% Search through files in data directory
colorFiles = dir(fullfile(inputFolder,'*.color.png'));
depthFiles = dir(fullfile(inputFolder,'*.regdepth.png'));
rawDepthFiles = dir(fullfile(inputFolder,'*.rawdepth.png'));
extFiles = dir(fullfile(inputFolder,'*.pose_camera_map.txt'));

% Load camera intrinsics
colorK = dlmread(fullfile(inputFolder,'color_intrinsics.K.txt'));
depthK = dlmread(fullfile(inputFolder,'depth_intrinsics.K.txt'));
depth2colorExt = [dlmread(fullfile(inputFolder,'depth2color_extrinsics.K.txt'));0,0,0,1];

% Load camera poses
extBin2WorldFile = dir(fullfile(inputFolder,'pose_bin*_map.txt'));
binNames = ['A','B','C','D','E','F','G','H','I','J','K','L'];
binId = binNames(str2num(extBin2WorldFile.name(9:(end-8)))+1);
extBin2World = dlmread(fullfile(inputFolder,extBin2WorldFile.name));
extWorld2Bin = inv(extBin2World);

fid = fopen(fullfile(outputFolder,'cam.info.txt'),'wb');
fprintf(fid,'# Environment: ');
if binId == -1
    fprintf(fid,'tote\n');
else
    fprintf(fid,'shelf\n');
    fprintf(fid,'# Bin ID: %s\n',binId);
end
fprintf(fid,'\n# Color camera intrinsic matrix\n')
fprintf(fid,'%15.8e\t %15.8e\t %15.8e\t\n',colorK');
fprintf(fid,'\n# Depth camera intrinsic matrix\n')
fprintf(fid,'%15.8e\t %15.8e\t %15.8e\t\n',depthK');
fprintf(fid,'\n# Depth-to-color camera extrinsic matrix\n')
fprintf(fid,'%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n',depth2colorExt');
fprintf(fid,'\n# World-to-bin transformation matrix\n')
fprintf(fid,'%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n',extBin2World');

mkdir(fullfile(outputFolder,'raw'));

% Copy over RGB-D images and save camera poses
for frameIdx = 1:length(colorFiles)
    colorImage = imread(fullfile(inputFolder,colorFiles(frameIdx).name));
    imwrite(colorImage,fullfile(outputFolder,sprintf('frame-%06d.color.png',frameIdx-1)));
    depthData = imread(fullfile(inputFolder,depthFiles(frameIdx).name));
    depthDataShifted = bitor(bitshift(depthData,3),bitshift(depthData,-13));
    imwrite(depthDataShifted,fullfile(outputFolder,sprintf('frame-%06d.depth.png',frameIdx-1)));
    rawDepthData = imread(fullfile(inputFolder,rawDepthFiles(frameIdx).name));
    rawDepthDataShifted = bitor(bitshift(rawDepthData,3),bitshift(rawDepthData,-13));
    imwrite(rawDepthDataShifted,fullfile(outputFolder,'raw',sprintf('frame-%06d.depth.png',frameIdx-1)));
    fprintf(fid,'\n# Camera-to-world extrinsic matrix (camera pose) for frame-%06d\n',frameIdx-1);
    fprintf(fid,'%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n',dlmread(fullfile(inputFolder,extFiles(frameIdx).name))');
end

fclose(fid);








% parse_pv_pairs: parses sets of property value pairs, allows defaults
% usage: params=parse_pv_pairs(default_params,pv_pairs)
%
% arguments: (input)
%  default_params - structure, with one field for every potential
%             property/value pair. Each field will contain the default
%             value for that property. If no default is supplied for a
%             given property, then that field must be empty.
%
%  pv_array - cell array of property/value pairs.
%             Case is ignored when comparing properties to the list
%             of field names. Also, any unambiguous shortening of a
%             field/property name is allowed.
%
% arguments: (output)
%  params   - parameter struct that reflects any updated property/value
%             pairs in the pv_array.
%
% Example usage:
% First, set default values for the parameters. Assume we
% have four parameters that we wish to use optionally in
% the function examplefun.
%
%  - 'viscosity', which will have a default value of 1
%  - 'volume', which will default to 1
%  - 'pie' - which will have default value 3.141592653589793
%  - 'description' - a text field, left empty by default
%
% The first argument to examplefun is one which will always be
% supplied.
%
%   function examplefun(dummyarg1,varargin)
%   params.Viscosity = 1;
%   params.Volume = 1;
%   params.Pie = 3.141592653589793
%
%   params.Description = '';
%   params=parse_pv_pairs(params,varargin);
%   params
%
% Use examplefun, overriding the defaults for 'pie', 'viscosity'
% and 'description'. The 'volume' parameter is left at its default.
%
%   examplefun(rand(10),'vis',10,'pie',3,'Description','Hello world')
%
% params = 
%     Viscosity: 10
%        Volume: 1
%           Pie: 3
%   Description: 'Hello world'
%
% Note that capitalization was ignored, and the property 'viscosity'
% was truncated as supplied. Also note that the order the pairs were
% supplied was arbitrary.



% function H=shadedErrorBar(x,y,errBar,lineProps,transparent)
%
% Purpose 
% Makes a 2-d line plot with a pretty shaded error bar made
% using patch. Error bar color is chosen automatically.
%
% Inputs
% x - vector of x values [optional, can be left empty]
% y - vector of y values or a matrix of n observations by m cases
%     where m has length(x);
% errBar - if a vector we draw symmetric errorbars. If it has a size
%          of [2,length(x)] then we draw asymmetric error bars with
%          row 1 being the upper bar and row 2 being the lower bar
%          (with respect to y). ** alternatively ** errBar can be a
%          cellArray of two function handles. The first defines which
%          statistic the line should be and the second defines the
%          error bar.
% lineProps - [optional,'-k' by default] defines the properties of
%             the data line. e.g.:    
%             'or-', or {'-or','markerfacecolor',[1,0.2,0.2]}
% transparent - [optional, 0 by default] if ==1 the shaded error
%               bar is made transparent, which forces the renderer
%               to be openGl. However, if this is saved as .eps the
%               resulting file will contain a raster not a vector
%               image. 
%
% Outputs
% H - a structure of handles to the generated plot objects.     
%
%
% Examples
% y=randn(30,80); x=1:size(y,2);
% shadedErrorBar(x,mean(y,1),std(y),'g');
% shadedErrorBar(x,y,{@median,@std},{'r-o','markerfacecolor','r'});    
% shadedErrorBar([],y,{@median,@std},{'r-o','markerfacecolor','r'});    
%
% Overlay two transparent lines
% y=randn(30,80)*10; x=(1:size(y,2))-40;
% shadedErrorBar(x,y,{@mean,@std},'-r',1); 
% hold on
% y=ones(30,1)*x; y=y+0.06*y.^2+randn(size(y))*10;
% shadedErrorBar(x,y,{@mean,@std},'-b',1); 
% hold off
%
%
% Rob Campbell - November 2009






% KINECTFUSIONDEMO Illustrates how to use the Kin2 to perform 3D
% reconstruction
%
% Note: You must add to the windows path the bin directory containing the 
%       Kinect20.Fusion.dll 
%       For example: C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\bin
%
% WARNING: KINECT FUSION FUNCTIONALITY IS STILL IN BETA
%          WE NEED TO FIX MEMORY LEAKAGE IN C++ CAUSING MATLAB TO CRASH AFTER
%          A SECOND RUN.
%
% Juan R. Terven, jrterven@hotmail.com
% Diana M. Cordova, diana_mce@hotmail.com
% 
% Citation:
% Terven Juan. Cordova-Esparza Diana, "Kin2. A Kinect 2 Toolbox for MATLAB", Science of
% Computer Programming, 2016. DOI: http://dx.doi.org/10.1016/j.scico.2016.05.009
%
% https://github.com/jrterven/Kin2, 2016.
















