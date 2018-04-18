addpaths;
ccc
%
% Part1: Test on Training Set
%
%% Part1: Test on training set
%
% This code is for testing trainig reward model on stages which
% had been used in the "TRAINING PHASE".
% This allows us to compare some effective statistics and
% derive "QUANTIATIVE" results.
%
ccc
global key_pressed
key_pressed = '';
alglist  = {'bmrl'};
% bmrl / gpirl / relent / maxent
modelist = {'safe_driving_mode', 'speedy_gonzales', 'tailgating_mode'};
% safe_driving_mode / speedy_gonzales / tailgating_mode
do_vidsave  = 0;
do_save      = 0;

for algitem = alglist
    algname = algitem{1};
    for modeitem = modelist
        modename = modeitem{1};
        fprintf('MODE: %s\n', modename);
        % Load trainig reward model
        % =============================================================================
        switch modename
            case 'safe_driving_mode', modeidx = 1;
            case 'speedy_gonzales', modeidx  = 2;
            case 'tailgating_mode', modeidx  = 3;
            case 'drunken_driving_mode', modeidx = 4;
            case 'mad_driving_mode', modeidx = 5;
        end
        loadname = sprintf('%s_%s.mat', algname, modename);
        lreward = load(loadname);
        fdir = dir([modename '/drive*']);
        ndir = length(fdir);
        
        % When you save video, only two cases will be used.
        if do_vidsave
            ndir = 2;
        end
        
        for i = 1:ndir % FOR EACH EPISODE
            
            fprintf('==========[%d/%d]==========\n', i, ndir);
            % For each scenario / Load
            loadname = [modename '/' fdir(i).name];
            l = load(loadname); rng(1);
            track = l.track;
            othercars = l.othercars;
            % Now, we have the stage.
            % Init simulation
            sim = init_sim(0.05); mode.flag = 1; save_flag = 0;
            % MyCar
            mycar.pos = [0 0 0]; mycar.vel = [0 0]; mycar.W = 5000; mycar.H = 3200;
            % Saver
            saver = init_saver();
            saverlist = init_saverlist('');
            % Run!
            clear plot_demo_test_trackbmrl;
            clear plot_demo_othercars;
            figpos = [100 500 1300 500]; axesinfo = [0.03, 0.03, 0.96, 0.9];
            fig = figure(1); clf;
            set(fig, 'KeyPressFcn', @keyDownListener, 'Position', figpos ...
                , 'MenuBar', 'none', 'NumberTitle', 'off' ...
                , 'Name', 'Track Driving Simulator');
            axes('Parent', fig, 'Position', axesinfo );
            start_lane_idx = 1;
            mycar.pos = get_posintrack(track, 1, 0, start_lane_idx, 0);
            while sim.flag
                iclk = clock;
                if isequal(key_pressed, '') == 0
                    switch key_pressed
                        case 'q' % Quit
                            sim.flag = 0;
                        case 'p' % Pause
                            mode.flag = ~mode.flag;
                        case 'r'
                            randlane = randi([1 track.nr_lane]);
                            mycar.pos = get_posintrack(track, 1, 0, randlane, 0);
                        case '1'
                            mycar.pos = get_posintrack(track, 1, 0, 1, 0);
                        case '2'
                            mycar.pos = get_posintrack(track, 1, 0, 2, 0);
                        case '3'
                            mycar.pos = get_posintrack(track, 1, 0, 3, 0);
                    end
                    % disp(key_pressed)
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
                        % Control mycar !!
                        carpos = mycar.pos;
                        [opt_vel, mycarpaths] ...
                            = car_controller(carpos, algname, track, othercars, lreward);
                        
                        % Some useful statistics
                        curr_vel = mycar.vel;
                        mycar.vel(1) = (2*opt_vel(1) + 0*curr_vel(1))/2;
                        mycar.vel(2) = (10*opt_vel(2) + 0*curr_vel(2))/10;
                        mycar.paths = mycarpaths;
                        % Get features of 'CURRENT CAR'
                        [myinfo, emsec_info] = get_trackinfo(track, mycar.pos, othercars);
                        mode.str = 'RUN';
                        % Saver
                        saver = add_saver(saver, mycar, myinfo);
                end
                
                % Plot
                plot_demo_test_trackbmrl(track, mycar, myinfo, sim.tick, sim.sec, sim.ems ...
                    , mode, modeidx);
                plot_demo_othercars(track, othercars);
                drawnow;
                if do_vidsave
                    img_name = sprintf('pics4vid/test_%s_%s_%02d_%03d.png' ...
                        , modename, algname, i, sim.tick);
                    % disp(img_name)
                    set(fig, 'PaperPositionMode','auto')
                    print(fig , '-dpng', img_name);
                end
                sim.ems = etime(clock, iclk)*1000;
                
                % Terminate condition
                feat = get_feat(myinfo, mycar.vel);
                if (isnan(sum(feat)) ...   % (If the car is out the track)
                        || abs(mycar.vel(1)) < 5000 && mycar.vel(2) < 5 ... % (If the car stops)
                        || mycar.pos(1) > track.xmax - 8000) ...
                        && sim.tick > 10
                    saverlist = add2saverlist(saverlist, saver, othercars);
                    fprintf('[%d/%d] lane: %d / %d-scenario terminated \n' ...
                        , i, ndir, start_lane_idx, saverlist.n);
                    start_lane_idx = start_lane_idx + 1;
                    if start_lane_idx > track.nr_lane
                        % Really terminate! move on to the next data
                        start_lane_idx = 1;
                        sim.flag = 0;
                        save_flag = 1;
                    end
                    mycar.pos = get_posintrack(track, 1, 0, start_lane_idx, 0);
                end
            end % while sim.flag
            
            if do_vidsave
                vidName = sprintf('vids/test_%s_%s_%02d.avi', modename, algname, i);
                frmRate = 40;
                video = VideoWriter( vidName );
                video.FrameRate = ( frmRate );
                open( video );
                for vididx = 1:sim.tick
                    img_name = sprintf('pics4vid/test_%s_%s_%02d_%03d.png' ...
                        , modename, algname, i, vididx);
                    img = imread(img_name);
                    img = im2double(img);
                    writeVideo( video, img );
                end
                close( video );
                fprintf(2, '%s saved. \n', vidName);
            end
            
            % SAVE: saverlist / track / othercars
            if save_flag && do_save
                temp = fdir(i).name(end-19:end-4);
                savename = sprintf('%s_result_%s.mat', algname, temp);
                save([modename '/' savename], 'saverlist', 'track', 'othercars');
                fprintf(2, ': %d data have been saved to %s \n', saverlist.n, savename);
            end
        end % for i = 1:ndir (for each scenario)
    end
end
fprintf(2, 'Done. \n');

%%
ccc
alglist  = {'bmrl', 'gpirl', 'relent', 'maxent'};
% bmrl / gpirl / relent / maxent
modelist = {'safe_driving_mode', 'speedy_gonzales', 'tailgating_mode'};
% safe_driving_mode / speedy_gonzales / tailgating_mode

do_vidsave = 0;
do_save    = 0;
i = 1;

for algitem = alglist
    algname = algitem{1};
    for modeitem = modelist
        modename = modeitem{1};
        fprintf('MODE: %s\n', modename);
        % Load trainig reward model
        % =============================================================================
        switch modename
            case 'safe_driving_mode', modeidx = 1;
            case 'speedy_gonzales', modeidx  = 2;
            case 'tailgating_mode', modeidx  = 3;
            case 'drunken_driving_mode', modeidx = 4;
            case 'mad_driving_mode', modeidx = 5;
        end
        loadname = sprintf('%s_%s.mat', algname, modename);
        lreward = load(loadname);
        for i = 1:2
            vidName = sprintf('vids/test_%s_%s_%02d.avi', modename, algname, i);
            frmRate = 10;
            
            img_temp = sprintf('pics4vid/test_%s_%s_%02d*' ...
                , modename, algname, i);
            fdir = dir(img_temp);
            ndir = length(fdir);
            
            video = VideoWriter( vidName );
            video.FrameRate = ( frmRate );
            open( video );
            for vididx = 1:5:ndir
                img_name = sprintf('pics4vid/test_%s_%s_%02d_%03d.png' ...
                    , modename, algname, i, vididx);
                img = imread(img_name);
                img = im2double(img);
                writeVideo( video, img );
            end
            close( video );
            fprintf(2, '%s saved. \n', vidName);
        end
    end
end
fprintf('done. \n');

%%

