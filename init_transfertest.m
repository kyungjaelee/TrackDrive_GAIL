function [lreward, sim, mode, modeidx, mycar, othercars, track, fig, saver] ...
    = init_transfertest(modename, algname, rseed, do_plot)

rng(rseed);
modeidx  = modename2idx(modename);
loadname = sprintf('%s_%s.mat', algname, modename);
lreward  = load(loadname);
uwidth   = 6000;
nr_lane  = 5;
width    = uwidth*nr_lane;
track    = init_track(nr_lane, width);
track    = set_track(track, 'simple');
othercars = init_othercars(track, 10, 'normal');
% Now, we have the stage. 
% Init simulation
sim = init_sim(0.1); mode.flag = 1;
% MyCar
mycar.pos = [0 0 0]; mycar.vel = [0 0]; mycar.W = 5000; mycar.H = 3200;
% Prepare for data-collection
saver = init_transfersaver(track);
% Run 
if do_plot
    fig = figure(1); clf;
    figpos = [400 300 900 900]; axesinfo = [0.03, 0.03, 0.96, 0.9];
    set(fig, 'KeyPressFcn', @keyDownListener, 'Position', figpos ...
        , 'MenuBar', 'none', 'NumberTitle', 'off', 'Name', 'Track Driving Simulator');
    axes('Parent', fig, 'Position', axesinfo );
else
    fig = '';
end