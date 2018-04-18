ccc
%% Part2: Test on Different Track (Transfer!)
ccc
global key_pressed
key_pressed = '';

% Load trainig reward model
modename = 'tailgating_mode';
% safe_driving_mode / speedy_gonzales / tailgating_mode
algname = 'bmrl';
% bmrl / gpirl / relent / maxent
% =============================================================================
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
saver = init_transfersaver();
% Run 
clear plot_demo_test_trackbmrl;
clear plot_demo_othercars;
fig = figure(1); clf;
figpos = [400 300 900 900]; axesinfo = [0.03, 0.03, 0.96, 0.9];
set(fig, 'KeyPressFcn', @keyDownListener, 'Position', figpos ...
    , 'MenuBar', 'none', 'NumberTitle', 'off', 'Name', 'Track Driving Simulator');
axes('Parent', fig, 'Position', axesinfo );
while sim.flag
    iclk = clock;
    if isequal(key_pressed, '') == 0
        switch key_pressed
            case 'q' % Quit
                sim.flag = 0;
            case 'p' % Pause
                mode.flag = ~mode.flag;
            case 'r'
                mycar.pos = get_posintrack(track, 1, 0, randi([1 track.nr_lane]), 0);
                othercars = init_othercars(track, 10, 'normal');
                clear plot_demo_test_trackbmrl;
                clear plot_demo_othercars;
                fig = figure(1); clf;
                set(fig, 'KeyPressFcn', @keyDownListener, 'Position', figpos ...
                    , 'MenuBar', 'none', 'NumberTitle', 'off', 'Name', 'Track Driving Simulator');
                axes('Parent', fig, 'Position', axesinfo );
        end
        key_pressed = '';
    end
    % Simulation starts here
    switch mode.flag
        case 0 % PAUSE
            mode.str = 'PAUSE';
        case 1 % RUN
            % Update
            sim = update_sim(sim);
            mycar.pos = update_pos(mycar.pos, mycar.vel, sim.T);
            for i = 1:othercars.n, othercars.car{i}.pos = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T); end
            % Control other carsqq
            [othercars, emsec_ctrl] = ctrl_cars(othercars, track);
            % Control mycar !!
            carpos = mycar.pos;
            [opt_vel, mycarpaths] = car_controller(carpos, algname, track, othercars, lreward);
            curr_vel = mycar.vel;
            mycar.vel(1) = (opt_vel(1) + curr_vel(1))/2;
            mycar.vel(2) = (9*opt_vel(2) + curr_vel(2))/10;
            mycar.paths = mycarpaths;
            % Get features of 'CURRENT CAR'
            [myinfo, emsec_info] = get_trackinfo(track, mycar.pos, othercars);
            mode.str = 'RUN';
            % Save statistics for analyze
            saver = save_transfer(saver, mycar, myinfo, othercars);
        otherwise
            fprintf(2, 'mode.flag should not be %d \n', mode.flag);
    end
    
    % Plot
    plot_demo_test_trackbmrl(track, mycar, myinfo, sim.tick, sim.sec, sim.ems, mode, modeidx);
    plot_demo_othercars(track, othercars);
    drawnow;
    sim.ems = etime(clock, iclk)*1000;
end
fprintf(2, 'Terminated.\n');

%%


