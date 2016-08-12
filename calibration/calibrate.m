% Calibrate camera poses for tote views
fprintf('Calibrating tote views...');
calibDirTote = '../data/benchmark/warehouse/calibration/tote';
getCalib(fullfile(calibDirTote,'seq-tote'),fullfile(calibDirTote,'cam.poses.txt'));
fprintf(' done!\n');

% Calibrate camera poses for all shelf bin views
fprintf('Calibrating shelf views...');
calibDirShelf = '../data/benchmark/warehouse/calibration/shelf';
binIds = 'ABCDEFGHIJKL';
for binIdx = 1:length(binIds)
    fprintf(sprintf(' %s',binIds(binIdx)));
    getCalib(fullfile(calibDirShelf,strcat('seq-',binIds(binIdx))),fullfile(calibDirShelf,sprintf('cam.poses.%s.txt',binIds(binIdx))));
end
fprintf(' done!\n');








