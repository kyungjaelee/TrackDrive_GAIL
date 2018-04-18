addpaths;
ccc;
%
% This code is for training policies
%
%% 1. BMRL (Proposed method)
ccc
algname = 'speedy_gonzales';
% safe_driving_mode / speedy_gonzales / tailgating_mode
% drunken_driving_mode / mad_driving_mode
fdir = dir([algname '/drive_record*']);
nmat = length(fdir);
fprintf('Loading %d mat file(s). \n', nmat);
totalfeat = [];
totallev  = [];
for i = 1:nmat % For all mat file
    fname = fdir(i).name; 
    l = load([algname '/' fname]);
    n = l.saverlist.n;
    for j = 1:n % For each episode
        curr_saver = l.saverlist.savers{j};
        curr_othercars = l.saverlist.othercars{j};
        curr_saver_n = curr_saver.n;
        curr_label   = curr_saver.label; % <= Positive or negative
        currfeats = [];
        for k = 1:curr_saver_n
            curr_myinfo = curr_saver.myinfo{k};
            curr_mycar = curr_saver.mycar{k};
            currfeat = get_feat(curr_myinfo, curr_mycar.vel);
            if isnan(sum(currfeat)) == 0 % if value is NOT NaN
                currfeats = [ currfeats ; currfeat];
                if curr_label == 1
                    totallev  = [totallev ; 1];
                else
                    totallev  = [totallev ; -1];
                end
            end
        end
        % Accumulate all data
        totalfeat = [totalfeat ; currfeats];
    end
end
dim = size(totalfeat, 2);
ntotal = size(totalfeat, 1);
fprintf('Total %d features. \n', ntotal);

% Add some noise
totalfeat = totalfeat + 0.001*randn(size(totalfeat));

% Run BMRL
Xd = totalfeat;
Ld = totallev;
lambda = 0.1;
beta   = 0.1;

lambda = 1;
beta   = 1;

% Median trick
if 0
    Nd = size(Xd, 1);
    dismat = cell(1, dim);
    deddist = zeros(1, dim);
    for i = 1:dim
        dismat{i} = pdist(Xd(randsample(Nd, min(1000, Nd)), i));
        deddist(i) = median(dismat{i});
    end
    mininvlen = 5;
    maxinvlen = 10;
    hypInit = [min(maxinvlen, max(mininvlen, deddist)) 1]';
else
    hypInit = [10*ones(1, dim) 1]';
end
opt  = struct('Display', 'on', 'LS_init', 1, 'LS', 1, 'Method', 'lbfgs' ...
    , 'MaxFunEvals', 4000, 'MaxIter', 1, 'TolFun', 1E-4, 'TolX', 1E-4, 'verbose', 0 ...
    , 'IndSet', 'track'); % track
[alphathat, hypOpt, Xu, Lu, nz] = bmrl(Xd, Ld, lambda, beta, hypInit, opt);


% Compute Reward at random location for testing
Xtest = rand(1, dim);
y = bmrl_reward(alphathat, hypOpt, Xu, Lu, nz, Xtest);

% Save
if 0
    save_name = sprintf('bmrl_%s.mat', algname);
    save(save_name, 'alphathat', 'hypOpt', 'Xu', 'Lu', 'nz');
    fprintf(2, 'BMRL results saved in %s \n', save_name);
end

%% 2. MaxEnt
ccc
rng(1);
% ============================================================== %
algname = 'safe_driving_mode';
% safe_driving_mode / speedy_gonzales / tailgating_mode
% drunken_driving_mode / mad_driving_mode
% ============================================================== %
% Load
l = load('track_mdp.mat');
mdp = l.mdp; mdp.discount = 0.7;
l = load(sprintf('%s/mdp_demo_F_%s.mat', algname, algname));
demo_info = l.demo_info;
% Initial parameter
feat_dim = get_feat();
% Function to minimize
fun2min = @(w)multiple_maxentopt( w, mdp, demo_info);
% Optimization parameters
verbose = 1; % <= !!
if verbose, dp = 'on'; else dp = 'off'; end
opt  = struct('Display', dp, 'LS_init', 2, 'LS', 2, 'Method', 'lbfgs' ...
    , 'MaxFunEvals', 50, 'MaxIter', 5, 'TolFun', 1E-5, 'TolX', 1E-10);
% Opimization with multiple restart
iclk = clock; best_nll = inf;
nrestart = 1;
for re_idx = 1:nrestart
    if verbose
        fprintf('Restart: [%d/%d] \n', re_idx, nrestart);
    end
    w0 = rand(feat_dim, 1);
    [wopt, nll] = minFunc(fun2min, w0, opt); % nll: smaller the better
    if verbose
        fprintf(2, 'OPT VAL: %.3e \n', -nll);
    end
    if best_nll > nll % store better solution
        best_nll = nll;
        best_wopt = wopt;
    end
end
ems_maxent = etime(clock, iclk)*1000;
% Final result!
disp(best_wopt');
fprintf('Took %.3f sec. \n', ems_maxent/1000);
% Save
savename = sprintf('maxent_%s.mat', algname);
save(savename, 'best_wopt', 'ems_maxent');
fprintf(2, 'MaxEnt results saved in %s \n', savename);

%% 3. GPIRL
ccc;
rng(1);
% ============================================================== %
algname = 'safe_driving_mode';
% safe_driving_mode / speedy_gonzales / tailgating_mode
% drunken_driving_mode / mad_driving_mode
% ============================================================== %
% Load
l = load('track_mdp.mat');
mdp = l.mdp; mdp.discount = 0.7;
l = load(sprintf('%s/mdp_demo_F_%s.mat', algname, algname));
demo_info = l.demo_info;
% Initial parameter
feat_dim = get_feat();
fdir = dir([algname '/drive_record*']);
nmat = length(fdir);
fprintf('Loading %d mat file(s). \n', nmat);
totalfeat = [];
totallev  = [];
for i = 1:nmat % For all mat file
    fname = fdir(i).name;
    l = load([algname '/' fname]);
    n = l.saverlist.n;
    for j = 1:n % For each episode
        curr_saver = l.saverlist.savers{j};
        curr_othercars = l.saverlist.othercars{j};
        curr_saver_n = curr_saver.n;
        curr_label   = curr_saver.label; % <= Positive or negative
        currfeats = [];
        for k = 1:curr_saver_n
            curr_myinfo = curr_saver.myinfo{k};
            curr_mycar = curr_saver.mycar{k};
            currfeat = get_feat(curr_myinfo, curr_mycar.vel);
            if isnan(sum(currfeat)) == 0 % if value is NOT NaN
                currfeats = [ currfeats ; currfeat];
                if curr_label == 1
                    totallev  = [totallev ; 1];
                else
                    totallev  = [totallev ; -1];
                end
            end
        end
        % Accumulate all data
        totalfeat = [totalfeat ; currfeats];
    end
end
dim = size(totalfeat, 2);
ntotal = size(totalfeat, 1);
fprintf('Total %d features. \n', ntotal);

% Add some noise
totalfeat = totalfeat + 0.001*randn(size(totalfeat));

% Run BMRL
Xd = totalfeat;
nz = init_nz(Xd);
[Xu,Lu] = get_indset(nz.data);
Xfs = cell(demo_info.n,1);
Lfs = cell(demo_info.n,1);
mu_sa_hat = cell(demo_info.n,1);
init_s = cell(demo_info.n,1);
for demo_idx = 1:demo_info.n
    Xfs{demo_idx}= get_nzval(nz,reshape(demo_info.F{demo_idx},[],feat_dim));
    Lfs{demo_idx} = ones(mdp.nstate*mdp.naction,1);
    
    demonstration = demo_info.demos{demo_idx};
    [ mu_sa_hat{demo_idx}, init_s{demo_idx} ] = datastatistics( mdp, demonstration );
end

% Function to minimize
fun2min = @(w)multiple_gpirlopt(w,Xu,Lu,Xfs,Lfs,mu_sa_hat,init_s,mdp,demo_info);
% Optimization parameters
verbose = 1; % <= !!
if verbose, dp = 'on'; else dp = 'off'; end
opt  = struct('Display', dp, 'LS_init', 2, 'LS', 2, 'Method', 'lbfgs' ...
    , 'MaxFunEvals', 100, 'MaxIter', 5, 'TolFun', 1E-5, 'TolX', 1E-10);
% Opimization with multiple restart
iclk = clock; best_nll = inf;
nrestart = 1;
for re_idx = 1:nrestart
    if verbose
        fprintf('Restart: [%d/%d] \n', re_idx, nrestart);
    end
    
    % super fucking black magic.......
    hyp = [.1; .1; .1; .1; .1; 10; 1];
    u = 0;
    Kuu = kernel_se(Xu,Xu,Lu,Lu,hyp);
    for demo_idx = 1:demo_info.n
        fprintf('restart[%d/%d] demo[%d/%d]\n', re_idx, nrestart, demo_idx, demo_info.n);
        Kfu = kernel_se(Xfs{demo_idx},Xu,Lfs{demo_idx},Lu,hyp);
        u = u + Kuu\(Kfu'*reshape(mu_sa_hat{demo_idx},[],1));
    end
    u = u/demo_info.n;
    u = u/max(abs(u));
    %     u = rand(size(Xu,1), 1);
    w0 = [u;hyp];
    
    % [wopt, nll] = minFunc(fun2min, w0, opt); % nll: smaller the better
    wopt = w0; nll = 0;
    
    if verbose
        fprintf(2, 'OPT VAL: %.3e \n', -nll);
    end
    if best_nll > nll % store better solution
        best_nll = nll;
        best_wopt = wopt;
    end
end
ems_gpirl = etime(clock, iclk)*1000;
% Final result!
disp(best_wopt');
fprintf('Took %.3f sec. \n', ems_gpirl/1000);
u_size = size(Xu,1);
u = best_wopt(1:u_size);
hyp = best_wopt(u_size+1:end);
[Kuu] = kernel_se(Xu, Xu, Lu, Lu, hyp);
[alpha] = gpirlsafeinv(Kuu,u);

% Save
savename = sprintf('gpirl_%s.mat', algname);
save(savename, 'alpha', 'Xu', 'Lu', 'nz','hyp','u');
fprintf(2, 'GPIRL results saved in %s \n', savename);

%% 4. RelEnt
ccc;
rng(1);
% ============================================================== %
algname = 'speedy_gonzales';
% safe_driving_mode / speedy_gonzales / tailgating_mode
% drunken_driving_mode / mad_driving_mode
% ============================================================== %
% Load
l = load('track_mdp.mat');
mdp = l.mdp; mdp.discount = 0.7;
l = load(sprintf('%s/mdp_demo_F_%s.mat', algname, algname));
demo_info = l.demo_info;
% Initial parameter
feat_dim = get_feat();
sample_params.demo_size = 2000;
sample_params.demo_len = 20;
baseline_demonstration = mdp_sample( mdp, @(s)randi(mdp.naction,1), sample_params );
base_mus = cell(sample_params.demo_size,1);
for base_idx = 1:sample_params.demo_size
    [ base_mus{base_idx} ] = datastatistics( mdp, baseline_demonstration(base_idx) );
end

mu_sa_hats = cell(demo_info.n,1);
for mdp_idx = 1:demo_info.n
    mu_sa_hats{mdp_idx} = datastatistics( mdp, demo_info.demos{mdp_idx} );
end

% Function to minimize
params.epsil = 1E-3;
fun2min = @(w)multiple_relentopt( w, mu_sa_hats, base_mus, mdp, demo_info, params );
% Optimization parameters
verbose = 1; % <= !!
if verbose, dp = 'on'; else dp = 'off'; end
opt  = struct('Display', dp, 'LS_init', 2, 'LS', 2, 'Method', 'lbfgs' ...
    , 'MaxFunEvals', 100, 'MaxIter',10, 'TolFun', 1E-5, 'TolX', 1E-10);
% Opimization with multiple restart
iclk = clock; best_nll = inf;
nrestart = 5;
for re_idx = 1:nrestart
    if verbose
        fprintf('Restart: [%d/%d] \n', re_idx, nrestart);
    end
    
    %
    w0 = rand(feat_dim,1);
    [wopt, nll] = minFunc(fun2min, w0, opt); % nll: smaller the better
    
    if verbose
        fprintf(2, 'OPT VAL: %.3e \n', -nll);
    end
    if best_nll > nll % store better solution
        best_nll = nll;
        best_wopt = wopt;
    end
end
ems_gpirl = etime(clock, iclk)*1000;
% Final result!
disp(best_wopt');
fprintf('Took %.3f sec. \n', ems_gpirl/1000);
 
% Save
savename = sprintf('relent_%s.mat', algname);
save(savename, 'best_wopt');
fprintf(2, 'RelEnt IRL results saved in %s \n', savename);

%%

