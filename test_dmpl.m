function [nCollision,avgLaneDeg,avgLaneDist,avgDirVel,outTrackFlag] ...
    = test_dmpl(dmpl,modeName,rSeed,max_sec,DO_PLOT)
global key_pressed

rng(rSeed);
uwidth   = 6000;
nr_lane  = 3;
width    = uwidth*nr_lane;
track    = init_track(nr_lane, width);
track    = set_track(track, 'simple');
othercars = init_othercars(track, 5, 'normal');
sim = init_sim(0.1); mode.flag = 1;
mycar.pos = [0 0 0]; mycar.vel = [0 0]; mycar.W = 5000; mycar.H = 3200;
saver = init_transfersaver(track,100);
if DO_PLOT
    clear_handlers()
    fig = figure(1); clf;
    figpos = [400 300 900 900]; axesinfo = [0.03, 0.03, 0.96, 0.9];
    set(fig, 'KeyPressFcn', @keyDownListener, 'Position', figpos ...
        , 'MenuBar', 'none', 'NumberTitle', 'off', 'Name', 'Track Driving Simulator');
    axes('Parent', fig, 'Position', axesinfo );
else
    fig = '';
end
while sim.flag % Simulate
    iclk = clock;
    if isequal(key_pressed, '') == 0
        switch key_pressed
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
            for i = 1:othercars.n, othercars.car{i}.pos ...
                    = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T); end
            % Control other cars
            [othercars, emsec_ctrl] = ctrl_cars(othercars, track);
            % Control mycar
            carpos = mycar.pos;
            [mycarpaths, ctrls, N, Kres] = get_randpaths_forwardOnly(carpos);
            costs = zeros(N, 1);
            for j = 1:N % For all paths
                cpath = mycarpaths(3*j-2:3*j, :)';
                cctrl = ctrls(2*j-1:2*j, :)';
                cost = 0;
                for k = round(linspace(Kres/4, Kres/2, 3))
                    cpos  = cpath(k, :);
                    cu    = cctrl(k, :);
                    cinfo = get_trackinfo(track, cpos, othercars);
                    feat  = get_feat(cinfo, cu);
                    temp = bmrl_reward(dmpl.alphathat ...
                        , dmpl.hypOpt, dmpl.Xu, dmpl.Lu ...
                        , dmpl.nz, feat);
                    % This is for handling last case
                    if k == Kres, temp = temp + temp*1; end
                    % Sum costs
                    cost  = cost + temp;
                end
                costs(j) = cost;
            end
            % Controller
            [~, optidx] = max(costs);
            opt_vel =  ctrls(2*optidx-1:2*optidx, 1)';
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
    end % switch mode.flag
    % Plot
    if DO_PLOT
        modeIdx = 2;
        plot_checkCollision(track,mycar,myinfo,sim.tick,sim.sec,sim.ems ...
            ,mode,modeIdx);
        plot_demo_othercars(track, othercars);
        drawnow;
    end
    sim.ems = etime(clock, iclk)*1000;
    % Terminate condition
    if sim.sec > max_sec || isnan(myinfo.lane_dev)
        % fprintf(' Current simulation finished. Collision: %d \n',  saver.nr_col);
        sim.flag = 0;
    end
end % while sim.flag

% Summarize
nCollision = saver.nr_col;
avgLaneDeg = mean(abs(saver.lane_deg(1:saver.n)));
avgLaneDist = mean(abs(saver.lane_dev(1:saver.n)));
avgDirVel = mean(saver.dir_vel(1:saver.n-1))*36*1e-4;
if isnan(avgLaneDeg)
    outTrackFlag = true;
else
    outTrackFlag = false;
end
