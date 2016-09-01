% Compile messages for Matlab ROS service pose_estimation
%
% Instructions:
%   1. Check this package is properly located in the src directory of your
%      ROS workspace (e.g. catkin_ws/src/pose_estimation/make.m)
%   2. Compile your ROS workspace in terminal with catkin_make
%   3. Source catkin_ws/devel/setup.sh
%   4. In Matlab, navigate to catkin_ws/src/pose_estimation
%   5. Run this script
%   6. If prompted, install Robotics Addons for Matlab
%   7. Follow the instructions to save javaclasspath.txt
%   8. Restart Matlab
%
% ---------------------------------------------------------
% Copyright (c) 2016, Andy Zeng
% 
% This file is part of the APC Vision Toolbox and is available 
% under the terms of the Simplified BSD License provided in 
% LICENSE. Please retain this notice and LICENSE if you use 
% this file (or any portion of it) in your project.
% ---------------------------------------------------------
cd(fileparts(which('make.m')));

% Save custom message files into catkin_ws/src/matlab_gen
currentPath = pwd;
slashStrIdx = strfind(currentPath,'/');
catkinWsPath = currentPath(1:(slashStrIdx(end-2)));
catkinWsSrcPath = fullfile(catkinWsPath,'src');

% Install Robotics Addons for ROS Custom Messages
if ~exist('rosgenmsg')
    fprintf('Please download and install add-ons for Matlab Robotics System Toolbox then restart this script.\n')
    roboticsAddons
    return;
end

% Restart Matlab ROS
try
    rosinit
catch
    rosshutdown
    rosinit
end

% Generate custom message files
if exist(fullfile(catkinWsSrcPath,'matlab_gen'),'file')
    rmdir(fullfile(catkinWsSrcPath,'matlab_gen'),'s');
end
rosgenmsg(catkinWsSrcPath);

% Move custom message files into current package directory
if exist('matlab_gen','file')
    rmdir('matlab_gen','s')
end
movefile(fullfile(catkinWsSrcPath,'matlab_gen'),'./');

% Need to copy over java class paths to the specified file
fprintf('\nCopy the following to javaclasspath.txt (typically in ~/.matlab/R2016x/javaclasspath.txt).\n');
ls(fullfile(pwd, 'matlab_gen/jar/*jar'));
input('Hit enter once you finished following the steps above.');

% Add custom message files to MATLAB path permanently
addpath(fullfile(pwd,'matlab_gen/msggen'));
savepath
fprintf('Finished installing messages. Please restart Matlab.\n');