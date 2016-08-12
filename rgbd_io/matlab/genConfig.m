close all; clear all;

pagePath = '~/apc/rgbd-annotator';
pageDir = dir(fullfile(pagePath,'src','*_*.js'));



fid = fopen(fullfile(pagePath,'webpack.config.template.js'),'rt') ;
pageConfig = fread(fid) ;
fclose(fid) ;
pageConfig = char(pageConfig.') ;

batchNum = 8;
portNum = 8010;

linkStr = 'URL';

for batchIdx = 1:batchNum
    configStr = [];

    for pageIdx = ((batchIdx-1)*300+1):(min((batchIdx)*300,length(pageDir)))
        configStr = strcat(configStr,sprintf(',\n    %s: "./src/%s.js"',pageDir(pageIdx).name(1:(end-3)),pageDir(pageIdx).name(1:(end-3))));
        linkStr = strcat(linkStr,sprintf(',\nhttp://visiongpu2.cs.princeton.edu:%d/%s.html',portNum,pageDir(pageIdx).name(1:(end-3))));
    end
    
    tmpPageConfig = strrep(pageConfig,'CONFIG_PLACEHOLDER',configStr);
    tmpPageConfig = strrep(tmpPageConfig,'PORT_PLACEHOLDER',num2str(portNum));
    fid = fopen(fullfile(pagePath,sprintf('webpack.config.%d.js',batchIdx)),'wt');
    fwrite(fid,tmpPageConfig);
    fclose(fid);
    
    portNum = portNum + 1;
    
end

fid = fopen('labelLinkList.csv','wt');
fwrite(fid,linkStr);
fclose(fid);









