ccc
%
% Part1: Generate an MDP structure which is common for all collected demonstrations
% Part2: Generate Feature Mapping of each 'mat' file.
%
%% Part1: Generate an MDP structure which is common for all collected demonstrations
%
% => track_mdp.mat
%
% This code is for converting track-driving world to MDP for further process with
% different IRL methods
%
ccc
foldername = 'safe_driving_mode';
% safe_driving_mode / speedy_gonzales / tailgating_mode
% drunken_driving_mode / mad_driving_mode
fdir = dir([foldername '/drive_record*']);
n = length(fdir);
l = load([foldername '/' fdir(1).name]);

% 1. First, generate MDP out of track information
track = l.saverlist.track;
xmin = track.xmin; xmax = track.xmax; ymin = track.ymin; ymax = track.ymax; dmin = 0; dmax = 360;
xres = 20; yres = 6; dres = 6;
opt_world = struct('type', 'xyd', 'xmin', xmin, 'xmax', xmax, 'ymin', ymin, 'ymax', ymax, 'dmin', dmin, 'dmax', dmax ...
    , 'xres', xres, 'yres', yres, 'dres', dres);
nstate = xres*yres*dres;
% Helper functions (State2Index and Index2State)
S2W = @(s)i2s(s, opt_world);
W2S = @(x)s2i(x, opt_world);
% Discretize control space
vmin = 0; vmax = 3E4; wmin = -60; wmax = 60;
vres = 4; wres =3 ;
opt_ctrl = struct('type', 'xy', 'xmin', vmin, 'xmax', vmax,'ymin', wmin, 'ymax', wmax ...
    , 'xres', vres, 'yres', wres);
naction = vres*wres;
% Helper functions
A2C = @(a)i2s(a, opt_ctrl);
C2A = @(u)s2i(u, opt_ctrl);

% Generate transition matrix (in this case, unicycle model)
T = 0.05; % time step
sig2 = [((xmax-xmin)/xres)^2, ((ymax-ymin)/yres)^2, ((dmax-dmin)/dres)^2]/3;
% Hueristic covariance matrix
grid_pnts = zeros(nstate,3);
for s = 1:nstate
    grid_pnts(s,:) = S2W(s);
end
t_p = zeros(nstate,naction,nstate); % transition prob
t_s = zeros(nstate,naction,nstate); % transition state
for s = 1:nstate
    for a = 1:naction
        pos = S2W(s);
        for k = 1:6
            pos = update_pos(pos, A2C(a), T);
        end
        t_s(s, a, :) = 1:nstate;
        t_p(s, a, :) = mvnpdf(grid_pnts, pos, diag(sig2));
        t_p(s, a, :) = t_p(s,a,:)/sum(t_p(s,a,:));
        % fprintf('%d,%d transition prob is computed!!! - max prob:%.2f\n',s,a,max(t_p(s,a,:)));
    end
end

% Make and Save MDP
mdp.discount = 0.9;
mdp.nstate = nstate;
mdp.naction = naction;
mdp.t_sa_s = t_s;
mdp.t_sa_p = t_p;
mdp.time_step = T;
mdp.uncertainty_dyn = sig2;
mdp.w2s = W2S;
mdp.s2w = S2W;
mdp.c2a = C2A;
mdp.a2c = A2C;
save('track_mdp.mat', 'mdp');
fprintf(2, 'MDP Saved! \n');

%% Part2: Generate Feature Mapping of each 'mat' file.
ccc
% ---------------------------------------------------- %
foldernamelist = {'safe_driving_mode', 'speedy_gonzales', 'tailgating_mode'};
% safe_driving_mode / speedy_gonzales / tailgating_mode
% drunken_driving_mode / mad_driving_mode
for foldernameitem = foldernamelist
    foldername = foldernameitem{1};
    % ------------------------------------------------ %
    l = load('track_mdp.mat');
    mdp = l.mdp;
    fdir = dir([foldername '/drive_record*']);
    n = length(fdir);
    demo_info.n = n; % <== We will save demonstrations and feature mapping matrix into this structure
    demo_info.F = cell(n, 1);
    demo_info.demos = cell(n, 1);
    for i = 1:n
        fname = fdir(i).name;
        l = load([foldername '/' fname]);
        dimfeat = get_feat();
        fprintf('[%d/%d] Loading %s \n', i, n, fname);
        
        % =======================================================================
        % Make feature mapping matrix
        % =======================================================================
        F = zeros(mdp.nstate, mdp.naction, dimfeat);
        for s = 1:mdp.nstate
            for a = 1:mdp.naction
                currinfo = get_trackinfo(l.track, mdp.s2w(s), l.othercars);
                F(s, a, :) = get_feat(currinfo, mdp.a2c(a)); % feature extraction
            end
        end
        fprintf(' Feature matrix generated. \n')
        %%
        % =======================================================================
        % Convert collected demonstrations to features
        % =======================================================================
        nr_demo = 0; total_len = 0;
        demonstration = cell(1000,1);
        for demo_idx = 1:l.saverlist.n
            % fprintf('  [%d/%d] \n', demo_idx, l.saverlist.n);
            curr_savers = l.saverlist.savers{demo_idx};
            curr_len = curr_savers.n;
            if curr_len > 10 % The length of the trajectory must exceeds '10'.
                trajs = zeros(curr_len, 2);
                nr_demo = nr_demo + 1;
                total_len = total_len + curr_len;
                for j = 1:curr_len
                    mycarpos = curr_savers.mycar{j}.pos;
                    mycarvel = curr_savers.mycar{j}.vel;
                    s = mdp.w2s(mycarpos);
                    a = mdp.c2a(mycarvel);
                    trajs(j, :) = [s, a];
                end
                
%                 for j = 1:curr_len-1
%                     mdp.t_sa_p(trajs(j, 1),trajs(j, 2),trajs(j+1, 1))
%                     [~,a] = max( mdp.t_sa_p(trajs(j, 1),:,trajs(j+1, 1)));
%                     trajs(j, 2) = a;
%                 end
                % Actual save is done here
                demonstration{nr_demo} = trajs';
            end
        end
        demonstration = demonstration(1:nr_demo); % Trim
        fprintf(' %d demos collected. total_len is %d \n', nr_demo, total_len);
        % Store
        demo_info.F{i} = F;
        demo_info.demos{i} = demonstration;
    end
    
    % Save to mat file
    savename = sprintf('%s/mdp_demo_F_%s.mat', foldername, foldername);
    save(savename, 'demo_info');
    fprintf(2, 'MDP feature mapping matrices and demonstrations saved to \n => %s. \n', savename);
end

%%
% addpath('./junk');
% [ mu_sa_hat, init_s ] = datastatistics( mdp,demo_info.demos{1});
% 
% mu_s_hat = sum(mu_sa_hat,2);
% vis_map = sum(reshape(mu_s_hat,xres,yres,dres),3);
% %%
% imagesc(vis_map)
% colorbar;












