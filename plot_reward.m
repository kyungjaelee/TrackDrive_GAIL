function fig = plot_reward(rew_opt)

% PLOT CONFIGURATION
PTYPE = 'P_FB';
FILL_LANES     = 0;
PLOTARROW      = 1;
COLORFUL_FLOWS = 0;

% INITIALIZE ENVIRONMENT
figsz    = [1 5 6.5 3.5]/10;
axespos  = [0.03, 0.05, 0.95, 0.84];

uwidth   = 6000;
nr_lane  = 4;
nothercars = 5;
width    = uwidth*nr_lane;
track    = init_track(nr_lane, width);
track    = set_track(track, 'auto_collect');

othercars = init_othercars(track, nothercars, 'stop');
% ############# FIGURE 1 - EPISTEMIC UNCERTAINTY #############
% fig = get_fig(figsz, 'Reward', axespos, struct('fignum', 1));

fig = figure(1); clf;
figpos = [300 300 1200 400]; axesinfo = [0.03, 0.03, 0.96, 0.9];
set(fig, 'KeyPressFcn', @keyDownListener, 'Position', figpos ...
    , 'MenuBar', 'none', 'NumberTitle', 'off', 'Name', 'Track Driving Simulator');
axes('Parent', fig, 'Position', axesinfo );
set(gcf,'Color', [1., 1., 1.] ); hold on;
xresmap   = 70; % 60
yresmap   = 23; % 23
xgridmap  = linspace(track.xmin+0, track.xmax-0, xresmap);
ygridmap  = linspace(track.ymin+0, track.ymax-0, yresmap);
[xmeshmap, ymeshmap] = meshgrid(xgridmap, ygridmap);
xymeshmap = [xmeshmap(:) ymeshmap(:)];
nmeshmap  = size(xymeshmap, 1);
nfeatsmap = [];
for i = 1:nmeshmap
    ccar.pos  = [xymeshmap(i, :), 0];
    cinfo = get_trackinfo(track, ccar.pos, othercars);
    cfeat  = get_feat(cinfo, 20000);
    nfeat = get_nzval(rew_opt.lnzr.nzr_x,cfeat);
    ncu = get_nzval(rew_opt.lnzr.nzr_y,20000);
    nfeatsmap = [nfeatsmap ; [nfeat,ncu]];
end
rew_map = get_rew(nfeatsmap,rew_opt.lrew);

pcolor(xgridmap, ygridmap ...
    , reshape(rew_map, yresmap, xresmap));
caxis([min(rew_map) max(rew_map)]);
colorbar; colormap jet; shading interp;
axisinfo =  plot_track_frame(track);
axis equal; axis(axisinfo); % grid on;
axis off;
plot_demo_othercars(track, othercars);
