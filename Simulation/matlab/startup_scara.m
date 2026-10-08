function root = startup_scara()
% Add only source folders, never generated output/cache folders.
root=fileparts(mfilename('fullpath'));
addpath(root,fullfile(root,'config'),fullfile(root,'simulink'), ...
    fullfile(root,'examples'),fullfile(root,'tests'));
addpath(genpath(fullfile(root,'src')));
% Keep Simulink-generated files out of source folders/current directory.
if ~isempty(which('Simulink.fileGenControl'))
    cache=fullfile(root,'output','cache');codegen=fullfile(root,'output','codegen');
    if ~isfolder(cache),mkdir(cache);end
    if ~isfolder(codegen),mkdir(codegen);end
    Simulink.fileGenControl('set','CacheFolder',cache,'CodeGenFolder',codegen);
end
end
