ccc;
%%
t = tcpip('127.0.0.1', 2002, 'NetworkRole','server');
while true
    fopen(t);
    data = fread(t, t.InputBufferSize);
    save_file_name = char(data)';
    fprintf('%s\n',save_file_name);
    %%
    
    % LOAD
    l  = load('./track_data.mat');
    lnzr.nzr_x.mu  = l.mu_Xd;
    lnzr.nzr_x.sig = l.std_Xd;
    lnzr.nzr_x.eps = 0;
    lnzr.nzr_y.mu  = l.mu_Yd;
    lnzr.nzr_y.sig = l.std_Yd;
    lnzr.nzr_y.eps = 0;
    lrew = load('./_reward_weights.mat');
    rew_opt = struct('lnzr',lnzr,'lrew',lrew);
    
    % Configuration
    maxDemo = 1e3; % Maximum number of demonstrations
    max_sec = 2;
    DO_PLOT = false;
    VERBOSE = false;
    
    % Auto-collect demonstrations
    saver = auto_collect_demonstrations(rew_opt,maxDemo,max_sec,DO_PLOT,VERBOSE);
    
    %
    
    track = saver.track;
    nfeats = [];
    nctrls = [];
    for i = 1:saver.n
        othercars = saver.othercars{i};
        cpos  = saver.mycar{i}.pos;
        cu    = saver.mycar{i}.vel;
        cinfo = get_trackinfo(track, cpos, othercars);
        feat  = get_feat(cinfo, cu);
        
        nfeat = get_nzval(lnzr.nzr_x,feat);
        ncu = get_nzval(lnzr.nzr_y,cu(:,1));
        
        if ~sum(isnan(feat))
            nfeats = [nfeats;nfeat];
            nctrls = [nctrls;ncu];
        end
    end
    trajs4TF = struct('observes',nfeats,'actions',nctrls);
    save('track_samples', 'trajs4TF');
    
    fwrite(t, 'Done');
    fprintf('Done');
    fclose(t);
    close all;
    
end
%%
