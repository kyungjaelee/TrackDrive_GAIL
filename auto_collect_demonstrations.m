function saver = auto_collect_demonstrations(rew_opt,maxDemo,max_sec,DO_PLOT,VERBOSE)

% Initialize track
rng('shuffle');
uwidth   = 6000;
nr_lane  = 4;
nothercars = 5;
width    = uwidth*nr_lane;
track    = init_track(nr_lane, width);
track    = set_track(track, 'auto_collect');

% Iniialize saver
saver = init_transfersaver(track,500);
coll_list = [];
timeout_list = [];
out_list = [];
while true
    
    % Initailize other cars
    othercars = init_othercars(track, nothercars, 'stop');
    sim = init_sim(0.1); mode.flag = 1;
    
    % Initialize mycar
    while true
        pos_opt.seg_idx = randi([1 track.nr_seg]);
        pos_opt.dist_offset = track.seg{pos_opt.seg_idx}.len*rand;
        pos_opt.lane_idx = randi([1 track.nr_lane]);
        pos_opt.devdeg_offset = 60*rand-30;
        pos_opt.devdist_offset = sign(randn)*0.5*track.lane_width*rand;
        tempPos = get_posintrack_adv(track,pos_opt);
        [myinfo, emsec_info] = get_trackinfo(track,tempPos,othercars);
        tempFdist = myinfo.center_fb_dists(1);
        tempBdist = myinfo.center_fb_dists(2);
        
        if (tempFdist > 15000) && (tempFdist < 40000) && (tempBdist > 10000)
            mycar = init_mycar(tempPos);
            break;
        end
    end
    if DO_PLOT
        clear_handlers()
        fig = figure(1); clf;
        figpos = [300 300 1200 400]; axesinfo = [0.03, 0.03, 0.96, 0.9];
        set(fig, 'KeyPressFcn', @keyDownListener, 'Position', figpos ...
            , 'MenuBar', 'none', 'NumberTitle', 'off', 'Name', 'Track Driving Simulator');
        axes('Parent', fig, 'Position', axesinfo );
    end
    
    while sim.flag % Simulate
        
        % Run
        iclk = clock;
        
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
            for k = round(linspace(Kres/4, Kres/2, 7))
                cpos  = cpath(k, :);
                cu    = cctrl(k, :);
                cinfo = get_trackinfo(track, cpos, othercars);
                feat  = get_feat(cinfo, cu);
                % This is where the reward function is computed!!!!
                % ---------------------------------------------------------
                nfeat = get_nzval(rew_opt.lnzr.nzr_x,feat);
                ncu = get_nzval(rew_opt.lnzr.nzr_y,cu(:,1));
                temp = get_rew([nfeat,ncu],rew_opt.lrew);
                if isnan(temp)
                    temp = -100.0;
                end
                % ---------------------------------------------------------
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
        
        % Plot
        if DO_PLOT
            modeIdx = 2;
            plot_checkCollision(track,mycar,myinfo,sim.tick,sim.sec,sim.ems,mode,modeIdx);
            plot_demo_othercars(track, othercars);
            drawnow;
        end
        sim.ems = etime(clock, iclk)*1000;
        
        % Check some condition...
        TIMEOUT_FLAG = false;
        if sim.sec > max_sec
            TIMEOUT_FLAG = true;
            if VERBOSE, fprintf(2,'TIMEOUT_FLAG. \n'); end
        end
        LAST_SEG_FLAG = false;
        if myinfo.seg_idx == track.nr_seg
            LAST_SEG_FLAG = true;
            if VERBOSE, fprintf(2,'LAST_SEG_FLAG. \n'); end
        end
        OUT_LANE_FLAG = false;
        if isnan(myinfo.lane_dev)
            OUT_LANE_FLAG = true;
            if VERBOSE, fprintf(2,'OUT_LANE_FLAG. \n'); end
        end
        COLLISION_FLAG = false;
        if saver.curr_col
            COLLISION_FLAG = true;
            if VERBOSE, fprintf(2,'COLLISION_FLAG. \n'); end
        end
        
        % Terminate condition
        if TIMEOUT_FLAG || LAST_SEG_FLAG || OUT_LANE_FLAG || COLLISION_FLAG
            % fprintf(' Current simulation finished. Collision: %d \n',  saver.nr_col);
            if VERBOSE
                fprintf('[%d] Demonstrations collected.\n',saver.n);
            end
            coll_list = [coll_list;COLLISION_FLAG];
            timeout_list = [timeout_list;TIMEOUT_FLAG];
            out_list = [out_list;OUT_LANE_FLAG];
            sim.flag = 0;
        end
        
    end % while sim.flag % Simulate
    
    % Terminimate if we collect a sufficient number of demonstrations
    if  saver.n > maxDemo
        fprintf ('Collected [%d] demonstrations. \n',saver.n);
        break;
    end
end

fprintf('[Coll %f, Time %f, Out %f]\n', mean(coll_list),mean(timeout_list),mean(out_list))