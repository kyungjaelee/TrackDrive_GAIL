addpaths;
ccc
%% Compute the collision ratio by varying the number of demonstrations
ccc
global key_pressed
key_pressed = '';

% Configuration
algName    = 'speedy_gonzales';
modeName   = 'Aggressive Driving Mode';
nDataList  = [1,10,20,50,100,500,1000];
VERBOSE    = false;
rSeedList  = 1:20;
max_sec    = 20;
DO_PLOT    = false;
DO_SAVE    = true;


% nDataList = [10];
% rSeedList = [1:10];
% DO_PLOT   = true;
% DO_SAVE   = true;


% Test over different number of data and random seeds
n_ndata = length(nDataList);
n_rseed = length(rSeedList);
for nIdx = 1:n_ndata % For different number of data
    % Parse
    nData = nDataList(nIdx);
    for rIdx = 1:n_rseed % For all random seeds
        % Parse
        rSeed = rSeedList(rIdx); 
        
        % Train DMPL
        dmpl = get_dmpl(algName,nData,rSeed,true,VERBOSE);
        
        % Test DMPL
        [nCollision,avgLaneDeg,avgLaneDist,avgDirVel,outTrackFlag] ...
            = test_dmpl(dmpl,modeName,rSeed,max_sec,DO_PLOT);
        
        % Print
        fprintf(['nData:[%d][%d/%d] rSeed:[%d][%d/%d] #Collision:[%d] LaneDevDeg:[%.2fdeg] OutTrack:[%d]' ...
            ' LaneDevDist:[%.2fmm] avgDirVel:[%.2fkm/h]\n'] ...
            ,nData,nIdx,n_ndata,rSeed,rIdx,n_rseed,nCollision,avgLaneDeg,avgLaneDist,avgDirVel,outTrackFlag);
        
        % Save
        if DO_SAVE
            saveName = sprintf('%s/dmplRes_nData%03d_rSeed%02d',algName,n_rseed);
            save(saveName,'nCollision','avgLaneDeg','avgLaneDist','avgDirVel','outTrackFlag');
            fprintf('[%s] Saved.\n',saveName);
        end
    end % for rIdx = 1:n_rseed % For all random seeds
end % for nIdx = 1:n_ndata % For different number of data
fprintf(2,'Done.\n');

%% 

















