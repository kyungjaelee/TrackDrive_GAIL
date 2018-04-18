function plot_makefigure...
     (track, mycar, info, tick, sec, emsec_total, mode, ctrl_mode)

persistent fist_flag h_cartraj h_cartrajbd h_carfill h_carsfill h_paths h_carsbd h_carsarrow h_carbd h_cararrow h_title;
if isempty(fist_flag)
     fist_flag = 1;
end

% My car boundary
mycar_bd = get_carshape(mycar.pos, mycar.W, mycar.H);
set(gcf,'Color', [0.6, 0.9, 0.8]/4 )

% Title
switch ctrl_mode
     case 1
          ctrl_name = 'Safe Driving Mode'; title_color = 'g'; mycar_col= [0.2 1 0.2];
     case 2
          ctrl_name = 'Speedy Gonzales'; title_color = 'r'; mycar_col= [1 0.2 0.2];
     case 3
          ctrl_name = 'Tail Gating Mode'; title_color = 'y'; mycar_col= 'y';
     case 4
          ctrl_name = 'Drunken Driving Mode'; title_color = 'm'; mycar_col= 'm';
     case 5
          ctrl_name = 'Mad Driving Mode'; title_color = 'c'; mycar_col= 'c';
end

% Plot
pred_hor = 5;
if fist_flag
     hold on;
     
     % Plot track
     axisinfo = plot_track_makefigure(track);
     
     % My Car pos
     % h_carfill = fill(mycar_bd(:, 1), mycar_bd(:, 2), mycar_col);
     % h_carbd = plot(mycar_bd(:, 1), mycar_bd(:, 2), 'k', 'LineWidth', 3);
     % [h1, h2, h3] = plot_arrow(mycar.pos, 2000, mycar.pos(3), 'k', 3);
     % h_cararrow = [h1 h2 h3];
     
     axis equal; axis(axisinfo); % grid on;
     axis off;
     xlabel('X [mm]', 'FontSize', 15); ylabel('Y [mm]', 'FontSize', 15);
     fist_flag = 0;
else
     
     % Car pos
     % h_carfill.Vertices = mycar_bd;
     % h_carfill.FaceColor = mycar_col;
     % h_carbd.XData = mycar_bd(:, 1); h_carbd.YData = mycar_bd(:, 2);
     % [x1, y1, x2, y2, x3, y3] = get_arrow(mycar.pos, 2000, mycar.pos(3), 'k', 3);
     % h_cararrow(1).XData = x1; h_cararrow(1).YData = y1;
     % h_cararrow(2).XData = x2; h_cararrow(2).YData = y2;
     % h_cararrow(3).XData = x3; h_cararrow(3).YData = y3;
     
end



function axisinfo = plot_track_makefigure(track)


% colors = flipud(jet(track.nr_seg)); % <= Colorful lanes
% colors = 0.2*ones(track.nr_seg, 3); % <= Gray lanes
colors = 0*ones(track.nr_seg, 3); % <= Black lanes


lw = 3;
arrowlen = 5000;
textoffset = 6500;

axisinfo = [inf -inf inf -inf];

for i = fliplr(1:track.nr_seg)
    % reverse order
    curr_pos = track.seg{i}.startpos;
    curr_bd = track.seg{i}.bd;
    curr_type = track.seg{i}.type;
    curr_color = colors(i, :);
    ps = track.seg{i}.p;
    % Corners
    p1 = ps(1, :); p2 = ps(2, :); p3 = ps(3, :); p4 = ps(4, :);
    
    % Fiell
    fill(curr_bd(:, 1), curr_bd(:, 2), 1*[1 1 1]);
    
    % Lanes
    nr_lane = track.nr_lane;
    switch curr_type
        case 'straight'
            for j = 1:nr_lane - 1
                a = j/(nr_lane)*p4 + (nr_lane-j)/(nr_lane)*p1;
                b = j/(nr_lane)*p3 + (nr_lane-j)/(nr_lane)*p2;
                xs = [a(1) a(2)]; ys = [b(1) b(2)];
                plot([a(1) b(1)], [a(2) b(2)], '-', 'Color', 'k', 'LineWidth', 2);
            end
        case 'right_turn'
            l1 = track.seg{i}.l1;
            l2 = flipud(track.seg{i}.l2);
            for j = 1:nr_lane - 1
                ltemp = j/(nr_lane)*l1 + (nr_lane-j)/(nr_lane)*l2;
                xs = ltemp(:, 1); ys = ltemp(:, 2);
                plot(ltemp(:, 1), ltemp(:, 2), '-', 'Color', 'k', 'LineWidth', 2);
            end
        case 'left_turn'
            l1 = track.seg{i}.l1;
            l2 = flipud(track.seg{i}.l2);
            for j = 1:nr_lane - 1
                ltemp = j/(nr_lane)*l1 + (nr_lane-j)/(nr_lane)*l2;
                plot(ltemp(:, 1), ltemp(:, 2), '-', 'Color', 'k', 'LineWidth', 2);
            end
    end
    
    % Start position
    plot_arrow(curr_pos(1:2), arrowlen/2, curr_pos(3), curr_color, 3);
    
    % Boudnary
    plot(curr_bd(:, 1), curr_bd(:, 2), '-', 'LineWidth', lw, 'Color', curr_color);
    
    % Center position for turn
    switch curr_type
        case 'straight'
            
        case 'right_turn'
            curr_centerpos = track.seg{i}.centerpos;
            plot(curr_centerpos(1), curr_centerpos(2), 'x', 'LineWidth', 2 ...
                , 'MarkerSize', 15, 'Color', curr_color);
        case 'left_turn'
            curr_centerpos = track.seg{i}.centerpos;
            plot(curr_centerpos(1), curr_centerpos(2), 'x', 'LineWidth', 2 ...
                , 'MarkerSize', 15, 'Color', curr_color);
    end
    
    % Get axis info from 'curr_bd'
    xs = curr_bd(:, 1);
    ys = curr_bd(:, 2);
    xmin = min(xs); xmax = max(xs);
    ymin = min(ys); ymax = max(ys);
    if xmin < axisinfo(1)
        axisinfo(1) = xmin;
    end
    if xmax > axisinfo(2)
        axisinfo(2) = xmax;
    end
    if ymin < axisinfo(3)
        axisinfo(3) = ymin;
    end
    if ymax > axisinfo(4)
        axisinfo(4) = ymax;
    end
end

% Text
for i = fliplr(1:track.nr_seg)
    curr_pos = track.seg{i}.startpos;
    text_pos = curr_pos(1:2) + textoffset*[cos(curr_pos(3)*pi/180) sin(curr_pos(3)*pi/180)];
    if i == 1 && 0
        text(text_pos(1), text_pos(2), sprintf('Start'), 'FontSize', 18 ...
            , 'HorizontalAlignment', 'Left', 'Color', 'r');

    else

    end
end

% Start line
%startp1 = track.seg{1}.p(1, :);
%startp4 = track.seg{1}.p(4, :);
%plot([startp1(1) startp4(1)], [startp1(2) startp4(2)], 'r-', 'LineWidth', 4);
