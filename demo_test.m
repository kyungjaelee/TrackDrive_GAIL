ccc;

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
maxDemo = 1.2e3; % Maximum number of demonstrations
max_sec = 4;
DO_PLOT = true;
VERBOSE = false;

% Auto-collect demonstrations
saver = auto_collect_demonstrations(rew_opt,maxDemo,max_sec,DO_PLOT,VERBOSE);

%%
ccc;
l  = load('./track_data.mat');
lnzr.nzr_x.mu  = l.mu_Xd;
lnzr.nzr_x.sig = l.std_Xd;
lnzr.nzr_x.eps = 0;
lnzr.nzr_y.mu  = l.mu_Yd;
lnzr.nzr_y.sig = l.std_Yd;
lnzr.nzr_y.eps = 0;
lrew = load('./_reward_weights.mat');
rew_opt = struct('lnzr',lnzr,'lrew',lrew);

plot_reward(rew_opt);