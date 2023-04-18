%% init path
filepath = which(mfilename);
filefolder = fileparts(filepath);
rootpath = fullfile(filefolder,'.');

load_bus(rootpath);


%% set build path
buildpath = fullfile(rootpath, 'build');
if(~exist(buildpath, 'dir'))
   mkdir(buildpath); 
end
set_param(0, 'CodeGenFolder', buildpath);
set_param(0, 'CacheFolder', buildpath);

%% add path
addpath(genpath('build'));