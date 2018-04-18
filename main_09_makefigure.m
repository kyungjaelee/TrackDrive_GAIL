ccc
addpaths

%%
ccc
global key_pressed
key_pressed = '';
once_flag = 1;
modenamelist = {'safe_driving_mode', 'speedy_gonzales', 'tailgating_mode'};
algnamelist  = {'bmrl'};
uwidth   = 6000;
nr_lane  = 3;
width    = uwidth*nr_lane;
track    = init_track(nr_lane, width);
track  = add_segment(track, 'straight', track.width, 25000);
track  = add_segment(track, 'right_turn', track.width, 20000);
track  = add_segment(track, 'straight', track.width, 5000);        
% Plot stuffs
fig = figure(1); clf;
figpos = [400 300 900 600]; axesinfo = [0.03, 0.03, 0.96, 0.9];
set(fig, 'KeyPressFcn', @keyDownListener, 'Position', figpos ...
    , 'MenuBar', 'none', 'NumberTitle', 'off', 'Name', 'Track Driving Simulator');
axes('Parent', fig, 'Position', axesinfo );
% colors = [0 1 0 ; 1 0.2 0.15 ; 1 1 0];
colors = lines(3);
ltypes = {'-', '--', ':'};
for modenameidx = 1:length(modenamelist) % For all driving modes
    modename = modenamelist{modenameidx};
    hs = zeros(3, 1);
    strs = cell(3, 1);
    for algnameidx = 1:length(algnamelist) % For all irl algorithms
        algname = algnamelist{algnameidx};
        fprintf('[%d/%d] modename: %s / algname: %s \n' ...
            , modenameidx, length(modenamelist), modename, algname);
        % Initialize the track-environment
        rng(1);
        loadname = sprintf('%s_%s.mat', algname, modename);
        lreward  = load(loadname);
        othercars = init_cars(5000, 3200);
        randpos =  get_posintrack(track, 2, 0, 1, 0);
        othercars = add_car(othercars, randpos, [0 0], 'stop');
        randpos =  get_posintrack(track, 3, 0, 2, 0);
        othercars = add_car(othercars, randpos, [0 0], 'stop');
        sim = init_sim(0.05); mode.flag = 1; mode.str = '';
        mycar.pos = [0 0 0]; mycar.vel = [0 0]; mycar.W = 5000; mycar.H = 3200;
        % Run
        while sim.flag
            if isequal(key_pressed, '') == 0
                switch key_pressed
                    case 'q'
                        sim.flag = 0;
                end
                key_pressed = '';
            end
            sim = update_sim(sim);
            mycar.pos = update_pos(mycar.pos, mycar.vel, sim.T);
            for i = 1:othercars.n, othercars.car{i}.pos ...
                    = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T); end
            % Control other carsqq
            [othercars, emsec_ctrl] = ctrl_cars(othercars, track);
            % Control mycar !!
            carpos = mycar.pos;
            [opt_vel, mycarpaths] ...
                = car_controller(carpos, algname, track, othercars, lreward);
            curr_vel = mycar.vel;
            mycar.vel(1) = (4*opt_vel(1) + curr_vel(1))/5;
            mycar.vel(2) = (9*opt_vel(2) + curr_vel(2))/10;
            mycar.paths = mycarpaths; 
            % Get features of 'CURRENT CAR'
            [myinfo, emsec_info] = get_trackinfo(track, mycar.pos, othercars);
            % Plot
            if once_flag
                once_flag = 0;
                plot_makefigure(track, mycar, myinfo, sim.tick, sim.sec, sim.ems, mode, 1);
                plot_demo_othercars(track, othercars);
                x1 = 0; x2 = 14000; y1 = -24000; y2 = -17000;
                bd = [x1 y1 ; x2 y1 ; x2 y2 ; x1 y2 ; x1 y1];
                fill(bd(:, 1), bd(:, 2), 1*[1 1 1 ]);
                mycar_bd1 = get_carshape([2000 -18500 0], mycar.W/2, mycar.H/2);
                mycar_bd2 = get_carshape([2000 -20500 0], mycar.W/2, mycar.H/2);
                mycar_bd3 = get_carshape([2000 -22500 0], mycar.W/2, mycar.H/2);
                plot(mycar_bd1(:, 1), mycar_bd1(:, 2) ...
                    , ltypes{1}, 'Color', colors(1, :), 'LineWidth', 2);
                plot(mycar_bd2(:, 1), mycar_bd2(:, 2) ...
                    , ltypes{2}, 'Color', colors(2, :), 'LineWidth', 2);
                plot(mycar_bd3(:, 1), mycar_bd3(:, 2) ...
                    , ltypes{3}, 'Color', colors(3, :), 'LineWidth', 2);
                text(4500, -18500, 'Safe driving mode', 'FontSize', 15);
                text(4500, -20500, 'Speedy Gonzales', 'FontSize', 15);
                text(4500, -22500, 'Tailgating mode', 'FontSize', 15); 
            end
            % current car
            mycar_bd = get_carshape(mycar.pos, mycar.W, mycar.H);
            if mod(sim.tick, 4) == 0
                hs(modenameidx) = plot(mycar_bd(:, 1), mycar_bd(:, 2) ...
                    , ltypes{modenameidx}, 'Color', colors(modenameidx, :)...
                    , 'LineWidth', 3);
            end
            drawnow;
            % Terminate
            if mycar.vel(1) < 200
                sim.flag = 0;
            end
        end
        fprintf(2, 'done. \n');
    end
end

fprintf(2, 'terminated. \n');

%%



%%