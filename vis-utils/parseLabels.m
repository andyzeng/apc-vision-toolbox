function entries = parseLabels(labelFile)
%PARSELABELS Summary of this function goes here
%   Detailed explanation goes here

fid = fopen(labelFile,'rb');
entries = {};
while ~feof(fid)
    labelLineText = fgets(fid);
    
    % Skip text lines without an object pose label
    if isempty(strfind(labelLineText,':['));
        continue;
    end
    
    % Parse sequence path, object name, and object label
    quoteIdx = strfind(labelLineText,'"');
    labelText = labelLineText((quoteIdx(1)+1):(quoteIdx(2)-1));
    entry.seqName = labelText(1:7);
    colonIdx = strfind(labelText,':');
    entry.objName = labelText((colonIdx(1)+1):(colonIdx(2)-1));
    bracketIdx = strfind(labelText,'[');
    objLabel = labelText((bracketIdx+1):(end-1));
    entry.objPose = reshape(str2num(char(strsplit(objLabel,','))),4,4);
    
    % Check label correctly corresponds to link
    if ~(size(strfind(labelLineText,entry.seqName),2) == 2) || ...
       ~(size(strfind(labelLineText,entry.objName),2) == 2)
        fprintf('Bad label at: %s',labelLineText(1:(end-1)));
        continue;
    end
    
    % Add to list of entries
    entries{length(entries)+1} = entry;

end
    
fclose(fid);












