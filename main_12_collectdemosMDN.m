ccc
global key_pressed; key_pressed = '';

% In this collecting process, we will make a simple and small track
% suitable for converting it to an MDP

% Init Track
uwidth  = 6000;
nr_lane = 4;
width   = uwidth*nr_lane;
track   = init_track(nr_lane, width);
track   = set_track(track, 'demonstration');

% Init MyCar and Other Cars
segidx = 1;
% laneidx = randi([1 track.nr_lane]);
laneidx = 1;
mycar.pos = get_posintrack(track, segidx, 0, laneidx, 0);
mycar.vel = [0 0]; mycar.W = 5000; mycar.H = 3200;
% Other cars
nr_othercars = randi([1 3]);
othercars = init_othercars(track, nr_othercars);

% Simulation
T = 0.02; % <= Don't change this.
sim = init_sim(T);

% Saver
saver = init_saver();
saverlist = init_saverlist(track);

% Figure
fig = figure(1);
axes('Parent', fig, 'Position', [0.01, 0.01, 0.98, 0.8] );
set(fig, 'KeyPressFcn', @keyDownListener, 'Position', [200 600 1400 700] ...
    , 'MenuBar', 'none', 'NumberTitle', 'off', 'Name', 'Track Driving Simulator');
sim.flag = true; 
save_flag = false; 
mode.flag = true; emsec_total = 0;
fprintf('Press\n [p]: postive save [n] negative save [r] reset [a] add current state to saver \n [w] save and quit [q] just quit \n');

while sim.flag
    iclk = clock;
    if isequal(key_pressed, '') == 0
        switch key_pressed
            case 'uparrow'
                mycar.vel(1) = mycar.vel(1) + 10000; % 5000[mm/s] -> 18km/s
            case 'downarrow'
                mycar.vel(1) = mycar.vel(1) - 10000;
            case 'leftarrow'
                mycar.vel(2) = mycar.vel(2) + 40;
            case 'rightarrow'
                mycar.vel(2) = mycar.vel(2) - 40;
            case 'space'
                mycar.vel = [0 0];
            case 's' % Make the car Straight!
                mycar.pos(3) = 0;
                mycar.vel(2) = 0;
            case 'q' % Just Quit
                sim.flag = 0;
                save_flag = 0;
            case 'w' % Quit + Write
                sim.flag = 0;
                    save_flag = 1;
            case 'p' % Positve
                if saver.n >= 3
                    saver.label = 1;
                    saverlist = add2saverlist(saverlist, saver, othercars);
                end
            case 'n' % Negative
                if saver.n >= 3
                    saver.label = 0;
                    saverlist = add2saverlist(saverlist, saver, othercars);
                end
            case 'r' % Reset
                
            case 'a' % Add current position
                saver = add_saver(saver, mycar, myinfo, 1);
            case 'u' % Undo
                if saverlist.n > 1
                    saverlist.n = saverlist.n - 1;
                end
            case '1'
                mycar.pos = get_posintrack(track, 1, 0, 1, 0);
                saver = init_saver();
            case '2'
                mycar.pos = get_posintrack(track, 1, 0, 2, 0);
                saver = init_saver();
            case '3'
                mycar.pos = get_posintrack(track, 1, 0, 3, 0);
                saver = init_saver();
            case '4'
                mycar.pos = get_posintrack(track, 1, 0, 4, 0);
                saver = init_saver();
        end
        if isequal(key_pressed, 'p') || isequal(key_pressed, 'n') || isequal(key_pressed, 'r')
            % mycar.pos = get_posintrack(track, randi([1 track.nr_seg]), 0, randi([1 nr_lane]), 0);
            mycar.pos = get_posintrack(track, 1, 0, laneidx, 0);
            mycar.vel = [0 0];
            saver = init_saver();
        end
        key_pressed = '';
    end
    switch mode.flag
        case 0 % PAUSE
            % Mode
            mode_str = 'PAUSE';
        case 1 % RUN
            % Update
            sim = update_sim(sim);
            mycar.pos = update_pos(mycar.pos, mycar.vel, T);
            for i = 1:othercars.n, othercars.car{i}.pos = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, T); end
            
            % Control other carsqq
            [othercars, emsec_ctrl] = ctrl_cars(othercars, track);
            
            % Get features
            [myinfo, emsec_info] = get_trackinfo(track, mycar.pos, othercars);
            
            % Useful features in meters and degrees
            geod      = myinfo.dist/1E3;
            dev       = myinfo.dev/1E3;
            ldev      = myinfo.lane_dev/1E3;
            ldeg      = myinfo.deg;
            fb_left   = myinfo.left_fb_dists/1E3;
            fb_right  = myinfo.right_fb_dists/1E3;
            fb_center = myinfo.center_fb_dists/1E3;
            if 0
                fprintf('GeodDist: %.2fm Dev: %.2fm LaneDev: %.2fm LaneDeg: %.2fdeg ' ...
                    , geod, dev, ldev, ldeg);
                fprintf('left: %.2fm %.2fm right: %.2fm %.2fm center: %.2fm %.2fm \n' ...
                    , fb_left(1), fb_left(2), fb_right(1), fb_right(2), fb_center(1), fb_center(2));
            end
            % Mode
            mode.str = 'RUN';
    end
    
    % Feature to save
    feat = get_feat(myinfo, mycar.vel);
    
    if isnan(sum(feat))
        % Goes outside!
        laneidx = laneidx + 1;
        fprintf('Saved! \n');
        key_pressed = 'p';
        if laneidx > track.nr_lane
            saver.label = 1;
            saverlist = add2saverlist(saverlist, saver, othercars);
            save_flag = true;
            sim.flag = false;
        end
    end
    
    % Saver
    saver = add_saver(saver, mycar, myinfo);
    
    % Plot
    iclk_plot = clock;
    plot_demo_train(track, mycar, myinfo, sim.tick, sim.sec, emsec_total, mode, saver, saverlist);
    plot_demo_othercars(track, othercars);
    drawnow;
    emsec_plot = etime(clock, iclk_plot)*1000;
    emsec_total = etime(clock, iclk)*1000;
    
end
title('Terminated', 'FontSize', 20, 'Color', 'r');
fprintf(2, 'Terminated. \n');

if save_flag
    savename = sprintf('drive_record_%s.mat', datestr(datenum(clock),'yyyy-mm-dd-HH-MM'));
    save(savename, 'saverlist', 'track', 'othercars');
    fprintf(2, ': %d data have been saved to %s \n', saverlist.n, savename);
end


%% Part2: Plot collected demos
ccc

% Save video
do_vidsave = 1;

modenamelist = {'safe_driving_mode', 'speedy_gonzales', 'tailgating_mode'};
for modenameidx = 1:length(modenamelist)
    modename = modenamelist{modenameidx};
    switch modename
        case 'safe_driving_mode', modeidx = 1; modetitle = 'Safe driving mode';
        case 'speedy_gonzales', modeidx  = 2; modetitle = 'Speedy Gonzales';
        case 'tailgating_mode', modeidx  = 3; modetitle = 'Tailgating mode';
        case 'drunken_driving_mode', modeidx = 4;
        case 'mad_driving_mode', modeidx = 5;
    end
    fdir = dir([modename '/drive_record*']);
    % fdir = dir([modename '/bmrl*']);
    nmat = length(fdir);
    fprintf('Loading %d mat file(s). \n', nmat);
    
    for i = 1:nmat
        fprintf('[%d/%d] \n', i, nmat);
        loadname = [modename '/' fdir(i).name];
        l = load(loadname);
        track = l.track;
        saverlist = l.saverlist;
        othercars = l.othercars;
        sim = init_sim(0.1);
        mycar.pos = [0 0 0]; mycar.vel = [0 0]; mycar.W = 5000; mycar.H = 3200; mycar.paths = [];
        myinfo = get_trackinfo(track, mycar.pos, othercars);
        mode.str = 'RUN'; mode.flag = 1;
        
        % Plot
        clear plot_demo_test_trackbmrl;
        clear plot_demo_othercars;
        fig = figure(i); clf;
        figpos = [100 500 1300 300]; axesinfo = [0.03, 0.03, 0.96, 0.9];
        set(fig, 'Position',  figpos);
        axes('Parent', fig, 'Position', axesinfo );
        hold on;
        plot_demo_test_trackbmrl(track, mycar, myinfo, sim.tick, sim.sec, sim.ems, mode, modeidx);
        plot_demo_othercars(track, othercars);
        colors = hsv(saverlist.n);
        for j = 1:saverlist.n
            alpha = 0.1;
            col = (1-alpha)*colors(j, :) + alpha*[0 0 0];
            saver = saverlist.savers{j};
            for k = 1:saver.n
                cpos = saver.mycar{k}.pos;
                cvel = saver.mycar{k}.vel;
                ccbd = get_carshape(saver.mycar{k}.pos, saver.mycar{k}.W, saver.mycar{k}.H);
                
                % plot(cpos(1), cpos(2), 'o', 'Color', 'g', 'MarkerSize', 15, 'LineWidth', 2);
                plot(ccbd(:, 1), ccbd(:, 2), 'Color', col, 'LineWidth', 2);
                plot_arrow(saver.mycar{k}.pos, 2000, saver.mycar{k}.pos(3), col, 3);
                
                if rand < 0.1
                    text(cpos(1), cpos(2), sprintf(' (%.1f, %.1f)', cvel(1), cvel(2)));
                end
            end
        end
        set(gcf,'Color', 'w' )
        title(sprintf('[%d/%d] %s', i, nmat, modetitle), 'FontSize', 20, 'Color', 'k');
        drawnow;
        
        % Save to image
        if do_vidsave
            set(fig,'PaperPositionMode','auto')
            imgname = sprintf('pics4vid/train_%s_%02d.png', modename, i);
            print (fig , '-dpng', imgname);
        else
            pause
        end
    end % for i = 1:nmat
    if do_vidsave
        vidName = sprintf('vids/train_%s.avi', modename);
        frmRate = 2;
        video = VideoWriter( vidName );
        video.FrameRate = ( frmRate );
        open( video );
        for vididx = 1:nmat
            imgname = sprintf('pics4vid/train_%s_%02d.png', modename, vididx);
            img = imread(imgname);
            img = im2double(img);
            writeVideo( video, img );
        end
        close( video );
        fprintf(2, '%s saved. \n', vidName);
    end
end
fprintf(2, 'Done. \n');

%%







