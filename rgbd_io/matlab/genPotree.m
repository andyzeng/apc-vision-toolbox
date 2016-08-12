


dataPath = '~/apc/rgbd-annotator/data/pointclouds/scenes';
dataDir = dir(fullfile(dataPath,'*.ply'));

cd ~/apc/rgbd-annotator/;
for seqIdx = 1:length(dataDir)
    sceneName = dataDir(seqIdx).name(1:(end-4));
    system(sprintf('./scripts/PotreeConverter/build/PotreeConverter/PotreeConverter ./data/pointclouds/scenes/%s.ply -o ./data --generate-page %s',sceneName,sceneName));
end















