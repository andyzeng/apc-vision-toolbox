




dataPath = '/home/andyzeng/apc/toolbox/data/benchmark/warehouse/practice/tote';
dataDir = dir(fullfile(dataPath,'old','seq-*.png'));



counter = 0;

for seqIdx = 1:length(dataDir)

    imPath = fullfile(dataPath,'old',dataDir(seqIdx).name);
    seqPath = fullfile(dataPath,'old',dataDir(seqIdx).name(1:(end-4)));

    newImPath = fullfile(dataPath,sprintf('seq-%04d.png',counter));
    newSeqPath = fullfile(dataPath,sprintf('seq-%04d',counter));
    counter = counter+1;

    [imPath,' ',newImPath]
    [seqPath,' ',newSeqPath]
    
    repIm = imread(imPath);
    if size(repIm,2) > 640
        imwrite([repIm(:,1:640,:);repIm(:,641:end,:)],newImPath);
        delete(imPath);
    else
        movefile(imPath,newImPath);
    end
    movefile(seqPath,newSeqPath);

end













