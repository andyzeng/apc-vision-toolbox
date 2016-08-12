

% saveToPath = '/home/andyzeng/apc/toolbox/data/benchmark/warehouse/practice';
% 
% dataPath = '/home/andyzeng/apcdata/warehouse/system_test';
% dataDir = dir(fullfile(dataPath,'0*'));
% 
% shelfCounter = 0;
% toteCounter = 0;
% 
% for seqIdx = 1:length(dataDir)
%     
%     seqPath = fullfile(dataPath,dataDir(seqIdx).name);
%     fprintf('%s\n',seqPath);
%     
%     numFrames = length(dir(fullfile(seqPath,'*.color.png')));
% 
%     if numFrames == 15
%         
%         newSeqPath = fullfile(saveToPath,'shelf',sprintf('seq-%04d',shelfCounter));
%         shelfCounter = shelfCounter + 1;
%         
%         convert(seqPath,newSeqPath,[newSeqPath,'.png']);
%         
%     elseif numFrames == 18
%         
%         newSeqPath = fullfile(saveToPath,'tote',sprintf('seq-%04d',toteCounter));
%         toteCounter = toteCounter + 1;
%         
%         convert(seqPath,newSeqPath,[newSeqPath,'.png']);
%         
%     end
%     
% end

convert('/home/andyzeng/apcdata/office/shelf_tote_empty/bin11',...
        '/home/andyzeng/apc/toolbox/data/benchmark/office/empty/shelf/seq-L',...
        '/home/andyzeng/apc/toolbox/data/benchmark/office/empty/shelf/seq-L.png');





















