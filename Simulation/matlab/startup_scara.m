function root = startup_scara()
% Source only; never add generated/cache directories to the MATLAB path.
root=fileparts(mfilename('fullpath'));
addpath(root,fullfile(root,'config'),fullfile(root,'simulink'),fullfile(root,'tests'));
addpath(genpath(fullfile(root,'src')));
if ~isempty(which('Simulink.fileGenControl'))
    cache=fullfile(root,'output','cache');codegen=fullfile(root,'output','codegen');
    if ~isfolder(cache),mkdir(cache);end
    if ~isfolder(codegen),mkdir(codegen);end
    Simulink.fileGenControl('set','CacheFolder',cache,'CodeGenFolder',codegen);
end
end