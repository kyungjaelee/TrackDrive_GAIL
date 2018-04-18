addpaths
ccc
%% Auto collect demonstrations with DMPL
ccc

% Train DMPL
algName = 'speedy_gonzales';
VERBOSE = true;
nData = 1e3;
rSeed = 1;
MED_TRICK = true;
dmpl = get_dmpl(algName,nData,rSeed,MED_TRICK,VERBOSE);


% Configuration
maxDemo = 1e2; % Maximum number of demonstrations
max_sec = 2; 
DO_PLOT = true;
VERBOSE = true;

% Auto-collect demonstrations
saver = auto_collect_demonstrations(dmpl,maxDemo,max_sec,DO_PLOT,VERBOSE);


fprintf(2,'Done.\n');

%%


