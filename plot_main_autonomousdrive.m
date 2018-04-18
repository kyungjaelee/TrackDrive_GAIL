function plot_main_autonomousdrive(track, mycar, othercars, sim, opts)

persistent first h_title h_carsfill h_carsbd h_carsarrow h_mycarfill h_mycarbd h_mycararrow h_mycarpaths
if isempty(first)
    first = 1;
end
% parse
myinfo = opts.myinfo;
mycarpaths = opts.mycarpaths;
nmycarpaths = size(mycarpaths, 1)/3;
mycarxypaths = cell(nmycarpaths, 1);
for i = 1:nmycarpaths
    mycarxypaths{i}.xpath = mycarpaths(3*i-2, :)';
    mycarxypaths{i}.ypath = mycarpaths(3*i-1, :)';
end
mycarpathccolors = opts.mycarpathccolors;
% plot pre-processing
othercars_bd = cell(othercars.n, 1);
for i = 1:othercars.n
    ccar = othercars.car{i};
    othercars_bd{i} = get_carshape(ccar.pos, ccar.W, ccar.H);
end
mycar_bd = get_carshape(mycar.pos, mycar.W, mycar.H);
tstr = sprintf(['[%d][%.2fsec] Autonomous Driving \n' ...
     ' [%dseg%dlane] dist: %.1fm dev: %.1fm lanedev: %.1fm deg: %.1fdeg \n' ...
     ' left dists: %.1fm %.1fm center dists: %.1fm %.1fm right dists: %.1fm %.1fm \n' ...
     ' One loop takes %.1fms'] ...
    , sim.tick, sim.sec, myinfo.seg_idx, myinfo.lane_idx ...
    , myinfo.dist/1000, myinfo.dev/1000, myinfo.lane_dev/1000, myinfo.deg ...
    , myinfo.left_fb_dists(1)/1000, myinfo.left_fb_dists(2)/1000 ...
    , myinfo.center_fb_dists(1)/1000, myinfo.center_fb_dists(2)/1000 ...
    , myinfo.right_fb_dists(1)/1000, myinfo.right_fb_dists(2)/1000 ...
    , sim.ems);

if first
    hold on
    % plot track and axes
    axisinfo = plot_track(track);
    xmin = axisinfo(1);
    ymin = axisinfo(3);
    plot([xmin xmin], [ymin ymin + 10000], 'w-', 'LineWidth', 3);
    plot([xmin xmin + 10000], [ymin ymin], 'w-', 'LineWidth', 3);
    text(xmin+10000, ymin-1000, 'X 10m', 'FontSize', 15, 'Color', 'w', 'HorizontalAlignment', 'Center')
    text(xmin, ymin+11000, 'Y 10m', 'FontSize', 15, 'Color', 'w', 'HorizontalAlignment', 'Center')
    % plot othercars
    othercarscols = lines(othercars.n);
    for i = 1:othercars.n
        h_carsfill{i} = fill(othercars_bd{i}(:, 1), othercars_bd{i}(:, 2), othercarscols(i, :));
        h_carsbd{i} = plot(othercars_bd{i}(:, 1), othercars_bd{i}(:, 2), 'Color', othercarscols(i, :), 'LineWidth', 3);
        [h1, h2, h3] = plot_arrow(othercars.car{i}.pos(1:2), 2000, othercars.car{i}.pos(3), 'w', 3);
        h_carsarrow{i} = [h1 h2 h3];
    end
    % plot mycar paths 
    for i = 1:nmycarpaths
        col = 'k';
        h_mycarpaths{i} = plot(mycarxypaths{i}.xpath, mycarxypaths{i}.ypath, '-', 'Color', mycarpathccolors(i, :));
    end
    % plot mycar
    h_mycarfill = fill(mycar_bd(:, 1), mycar_bd(:, 2), 'g');
    h_mycarbd = plot(mycar_bd(:, 1), mycar_bd(:, 2), 'k', 'LineWidth', 3);
    [h1, h2, h3] = plot_arrow(mycar.pos, 2000, mycar.pos(3), 'k', 3);
    h_mycararrow = [h1 h2 h3];
    % plot title and axis
    h_title = title(tstr, 'FontSize', 20, 'Color', 'w');
    axis equal off
    axis(axisinfo - [1000 0 0 0]);
    
    first = 0;
else
    % plot other cars
    for i = 1:othercars.n
        h_carsfill{i}.Vertices = othercars_bd{i};
        h_carsbd{i}.XData = othercars_bd{i}(:, 1);
        h_carsbd{i}.YData = othercars_bd{i}(:, 2);
        [x1, y1, x2, y2, x3, y3] = get_arrow(othercars.car{i}.pos, 2000, othercars.car{i}.pos(3), 'k', 3);
        h_carsarrow{i}(1).XData = x1; h_carsarrow{i}(1).YData = y1;
        h_carsarrow{i}(2).XData = x2; h_carsarrow{i}(2).YData = y2;
        h_carsarrow{i}(3).XData = x3; h_carsarrow{i}(3).YData = y3;
    end 
    % plot mycar paths 
    for i = 1:nmycarpaths
        h_mycarpaths{i}.XData = mycarxypaths{i}.xpath;
        h_mycarpaths{i}.YData = mycarxypaths{i}.ypath;
        h_mycarpaths{i}.Color = mycarpathccolors(i, :);
    end
    % plot my car 
    h_mycarfill.Vertices = mycar_bd;
    h_mycarbd.XData = mycar_bd(:, 1);
    h_mycarbd.YData = mycar_bd(:, 2);
    [x1, y1, x2, y2, x3, y3] = get_arrow(mycar.pos, 2000, mycar.pos(3), 'k', 3);
    h_mycararrow(1).XData = x1; h_mycararrow(1).YData = y1;
    h_mycararrow(2).XData = x2; h_mycararrow(2).YData = y2;
    h_mycararrow(3).XData = x3; h_mycararrow(3).YData = y3;
    % plot title
    h_title.String = tstr;
end
drawnow;
