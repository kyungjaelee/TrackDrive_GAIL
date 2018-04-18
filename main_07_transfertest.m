addpaths;
ccc
%% Comparative Test on Transfered Track!
ccc
global key_pressed
key_pressed = '';

modenamelist = {'safe_driving_mode', 'speedy_gonzales', 'tailgating_mode'}; 

modenamelist = {'speedy_gonzales'};

% {'safe_driving_mode', 'speedy_gonzales', 'tailgating_mode'};
algnamelist = {'bmrl'};
% {'bmrl', 'gpirl', 'relent', 'maxent'};
rseedlist  = 5;  % 1:10
max_sec    = 60; % 60
do_plot    = 1;  % Plot? 
do_vidsave = 0;
do_save    = 0;
% =============================================================================
nrmode = length(modenamelist);
nralg = length(algnamelist);
nrrseed = length(rseedlist);
for modenameidx = 1:nrmode % For all driving modes
    modename = modenamelist{modenameidx};
    for algnameidx = 1:nralg % For all irl algorithms
        algname = algnamelist{algnameidx};
        for rseedidx = 1:nrrseed % For all random seeds
            rseed = rseedlist(rseedidx);
            fprintf('[%d/%d][%d/%d][%d/%d] modename: %s algname: %s rseed: %d \n' ...
                , modenameidx, nrmode, algnameidx, nralg, rseedidx, nrrseed ...
                , modename, algname, rseedidx);
            % Init sim
            clear plot_demo_test_trackbmrl plot_demo_othercars
            [lreward, sim, mode, modeidx, mycar, othercars, track, fig, saver] ...
                = init_transfertest(modename, algname, rseed, do_plot);
            while sim.flag
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
                end % switch mode.flag
                % Plot
                if do_plot 
                    plot_demo_test_trackbmrl(track, mycar, myinfo, sim.tick, sim.sec, sim.ems, mode, modeidx);
                    plot_demo_othercars(track, othercars);
                    drawnow;
                    if do_vidsave
                        img_name = sprintf('pics4vid/%s_%s_%02d_%03d.png', modename, algname, rseed, sim.tick);
                        set(fig, 'PaperPositionMode','auto')
                        print(fig , '-dpng', img_name);
                    end
                end
                sim.ems = etime(clock, iclk)*1000;
                % Terminate condition
                if sim.sec > max_sec
                    fprintf(' Current simulation finished. Collision: %d \n',  saver.nr_col);
                    sim.flag = 0;
                end
            end % while sim.flag
            % Save here
            if do_save
                % ymdhm = datestr(datenum(clock),'yyyy-mm-dd-HH-MM');
                savename = sprintf('%s/%s_transfer_%03d.mat', modename, algname, rseed);
                save(savename, 'saver');
                fprintf('%s Saved \n', savename);
            end
            if do_vidsave
                vidName = sprintf('vids/%s_%s_%02d.avi', modename, algname, rseed);
                frmRate = ceil(1/sim.T);
                video = VideoWriter( vidName );
                video.FrameRate = ( frmRate );
                open( video );
                for vididx = 1:sim.tick
                    img_name = sprintf('pics4vid/%s_%s_%02d_%03d.png', modename, algname, rseed, vididx);
                    img = imread(img_name);
                    img = im2double(img);
                    writeVideo( video, img );
                end
                close( video );
                fprintf(2, '%s saved. \n', vidName);
            end
        end % for rseedidx = 1:nrrseed % For all random seeds
    end % for algnameidx = 1:nralg % For all irl algorithms
end % for modenameidx = 1:nrmode % For all driving modes
fprintf(2, 'Terminated.\n');

%% SAVE VIDEO
ccc
modenamelist = {'safe_driving_mode', 'speedy_gonzales', 'tailgating_mode'}; 
% {'safe_driving_mode', 'speedy_gonzales', 'tailgating_mode'};
algnamelist = {'bmrl', 'gpirl', 'relent', 'maxent'};
% {'bmrl', 'gpirl', 'relent', 'maxent'};
rseedlist  = 1:2;  % 1L10

nrmode = length(modenamelist);
nralg = length(algnamelist);
nrrseed = length(rseedlist);
for modenameidx = 1:nrmode % For all driving modes
    modename = modenamelist{modenameidx}; 
    for algnameidx = 1:nralg % For all irl algorithms
        algname = algnamelist{algnameidx};
        for rseedidx = 1:nrrseed % For all random seeds
            rseed = rseedlist(rseedidx);
            vidName = sprintf('vids/%s_%s_%02d.avi', modename, algname, rseed);
            
            imgname = sprintf('pics4vid/%s_%s_%02d', modename, algname, rseed);
            fdir = dir([imgname '*']);
            nimgs = length(fdir);
            
            frmRate = 10;
            video = VideoWriter( vidName );
            video.FrameRate = ( frmRate );
            open( video );
            for vididx = 1:3:nimgs
                img_name = sprintf('pics4vid/%s_%s_%02d_%03d.png', modename, algname, rseed, vididx);
                img = imread(img_name);
                img = im2double(img);
                writeVideo( video, img );
            end
            close( video );
            fprintf(2, '%s saved. \n', vidName);
        end
    end
end
fprintf('done.\n');

%%







