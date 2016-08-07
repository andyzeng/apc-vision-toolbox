% Calibrate camera-to-world camera poses for the tote views
fprintf('Calibrating tote views...');
calibDirTote = '../data/benchmark/office/calibration/tote';
calibSeq(fullfile(calibDirTote,'seq-tote'),fullfile(calibDirTote,'cam.poses.txt'));
fprintf(' done!\n');

% Calibrate camera-to-world camera poses for the shelf views
fprintf('Calibrating shelf views...');
calibDirShelf = '../data/benchmark/office/calibration/shelf';
fprintf(' A'); calibSeq(fullfile(calibDirShelf,'seq-A'),fullfile(calibDirShelf,'cam.poses.A.txt'));
fprintf(' B'); calibSeq(fullfile(calibDirShelf,'seq-B'),fullfile(calibDirShelf,'cam.poses.B.txt'));
fprintf(' C'); calibSeq(fullfile(calibDirShelf,'seq-C'),fullfile(calibDirShelf,'cam.poses.C.txt'));
fprintf(' D'); calibSeq(fullfile(calibDirShelf,'seq-D'),fullfile(calibDirShelf,'cam.poses.D.txt'));
fprintf(' E'); calibSeq(fullfile(calibDirShelf,'seq-E'),fullfile(calibDirShelf,'cam.poses.E.txt'));
fprintf(' F'); calibSeq(fullfile(calibDirShelf,'seq-F'),fullfile(calibDirShelf,'cam.poses.F.txt'));
fprintf(' G'); calibSeq(fullfile(calibDirShelf,'seq-G'),fullfile(calibDirShelf,'cam.poses.G.txt'));
fprintf(' H'); calibSeq(fullfile(calibDirShelf,'seq-H'),fullfile(calibDirShelf,'cam.poses.H.txt'));
fprintf(' I'); calibSeq(fullfile(calibDirShelf,'seq-I'),fullfile(calibDirShelf,'cam.poses.I.txt'));
fprintf(' J'); calibSeq(fullfile(calibDirShelf,'seq-J'),fullfile(calibDirShelf,'cam.poses.J.txt'));
fprintf(' K'); calibSeq(fullfile(calibDirShelf,'seq-K'),fullfile(calibDirShelf,'cam.poses.K.txt'));
fprintf(' L'); calibSeq(fullfile(calibDirShelf,'seq-L'),fullfile(calibDirShelf,'cam.poses.L.txt'));
fprintf(' done!\n');








