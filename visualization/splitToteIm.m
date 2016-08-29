
% name = 'wpt0094';
% 
% I1 = imread(strcat('/home/andyz/apc/paper/figures/benchmark_sample/',name,'.pose.png'));
% I2 = imread(strcat('/home/andyz/apc/paper/figures/benchmark_sample/',name,'.seg.png'));
% 
% imwrite(I1(1:480,:,:),strcat('/home/andyz/apc/paper/figures/benchmark_sample/',name,'_1.pose.png'));
% imwrite(I1(481:end,:,:),strcat('/home/andyz/apc/paper/figures/benchmark_sample/',name,'_2.pose.png'));
% 
% imwrite(I2(1:480,:,:),strcat('/home/andyz/apc/paper/figures/benchmark_sample/',name,'_1.seg.png'));
% imwrite(I2(481:end,:,:),strcat('/home/andyz/apc/paper/figures/benchmark_sample/',name,'_2.seg.png'));






depth = imread('/home/andyz/apc/toolbox/rolodex_jumbo_pencil_cup/000010/frame-000005.rawdepth.png');

imshow(double(depth)./10000);
colormap jet















