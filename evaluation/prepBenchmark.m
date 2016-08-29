



seqList = {};
benchmarkPath = '/home/andyz/apc/toolbox/data/benchmark';
fid = fopen(fullfile(benchmarkPath,'info.txt'),'wb');

% otsDir = dir(fullfile(benchmarkPath,'office/test/shelf/scene-0*'));
% for seqIdx = 1:length(otsDir)
%     fprintf(fid,'office/test/shelf/%s,\n',otsDir(seqIdx).name);
% end
% 
% ottDir = dir(fullfile(benchmarkPath,'office/test/tote/scene-0*'));
% for seqIdx = 1:length(ottDir)
%     fprintf(fid,'office/test/tote/%s,\n',ottDir(seqIdx).name);
% end
% 
% wpsDir = dir(fullfile(benchmarkPath,'warehouse/practice/shelf/scene-0*'));
% for seqIdx = 1:length(wpsDir)
%     fprintf(fid,'warehouse/practice/shelf/%s,\n',wpsDir(seqIdx).name);
% end
% 
% wptDir = dir(fullfile(benchmarkPath,'warehouse/practice/tote/scene-0*'));
% for seqIdx = 1:length(wptDir)
%     fprintf(fid,'warehouse/practice/tote/%s,\n',wptDir(seqIdx).name);
% end

wcsDir = dir(fullfile(benchmarkPath,'warehouse/competition/shelf/scene-0*'));
for seqIdx = 1:length(wcsDir)
    fprintf(fid,'warehouse/competition/shelf/%s,\n',wcsDir(seqIdx).name);
end

wctDir = dir(fullfile(benchmarkPath,'warehouse/competition/tote/scene-0*'));
for seqIdx = 1:length(wctDir)
    fprintf(fid,'warehouse/competition/tote/%s,\n',wctDir(seqIdx).name);
end

fclose(fid);    
    
    
    
    
    
    
    
    
    