function plot_demo_othercars(track, othercars)

persistent fist_flag h_carsfill h_carsbd h_carsarrow h_paths h_texts h

show_text = 0;

if isempty(fist_flag)
    fist_flag = 1;
end

% Other cars boundary
cars_bd = cell(othercars.n, 1);
for i = 1:othercars.n
    ccar = othercars.car{i};
    % cars_bd{i} = get_circle(ccar.pos(1), ccar.pos(2), 2000, 20);
    cars_bd{i} = get_carshape(ccar.pos, ccar.W, ccar.H);
end

real_carwh  = [ccar.W*1.6, ccar.H*1.4];
rszwh  = [50 100];

% Plot
pred_hor = 5;
if fist_flag
    hold on;
    % Other cars
    othercarscols = lines(othercars.n);
    for i = 1:othercars.n
        % col = othercarscols(i, :);
        col = [0.9 0.2 0.6];
        
        randpaths = othercars.car{i}.paths;
        npath = size(randpaths, 1)/3;
        if npath > 0
            cpaths = [];
            for j = 1:npath
                cpath = randpaths(3*j-2:3*j, :)';
                cpaths = [cpaths ; NaN NaN ; cpath(:, 1:2)];
            end
            h_paths{i} = plot(cpaths(:, 1), cpaths(:, 2), '-', 'Color', col, 'LineWidth', 2);
        end
        if 1
            [carimg, ~, cartr] = imread('carimgs/whitecar.png');
            h.carrsz = imresize(carimg, rszwh);
            h.trrsz  = imresize(cartr, rszwh);
            carpos = othercars.car{i}.pos;
            [xmesh_grid, ymesh_grid] = get_carmeshgrid(carpos, real_carwh, rszwh);
            h.carsurf{i} = surf('xdata', xmesh_grid, 'ydata', ymesh_grid ...
                , 'zdata', zeros(rszwh(1), rszwh(2)) ...
                , 'cdata', h.carrsz, 'AlphaData', h.trrsz ...
                , 'FaceAlpha', 'texture' ...
                , 'FaceColor', 'texturemap' ...
                , 'EdgeColor','None', 'LineStyle', 'None');
        else
            h_carsfill{i} = fill(cars_bd{i}(:, 1), cars_bd{i}(:, 2), col);
            h_carsbd{i} = plot(cars_bd{i}(:, 1), cars_bd{i}(:, 2), 'Color', col, 'LineWidth', 3);
            [h1, h2, h3] = plot_arrow(othercars.car{i}.pos(1:2), 2000, othercars.car{i}.pos(3), 'w', 3);
            h_carsarrow{i} = [h1 h2 h3];
        end
        if show_text
            h_texts{i} = text(othercars.car{i}.pos(1), othercars.car{i}.pos(2), sprintf('%d', i) ...
                , 'FontSize', 13, 'HorizontalAlignment', 'Center');
        end
    end
    
    fist_flag = 0;
else
    
    % Other cars
    for i = 1:othercars.n
        if 1
            carpos = othercars.car{i}.pos;
            [xmesh_grid, ymesh_grid] = get_carmeshgrid(carpos, real_carwh, rszwh);
            h.carsurf{i}.XData = xmesh_grid;
            h.carsurf{i}.YData = ymesh_grid;
        else
            h_carsfill{i}.Vertices = cars_bd{i};
            h_carsbd{i}.XData = cars_bd{i}(:, 1);
            h_carsbd{i}.YData = cars_bd{i}(:, 2);
        end
        [x1, y1, x2, y2, x3, y3] = get_arrow(othercars.car{i}.pos, 2000, othercars.car{i}.pos(3), 'k', 3);
        h_carsarrow{i}(1).XData = x1; h_carsarrow{i}(1).YData = y1;
        h_carsarrow{i}(2).XData = x2; h_carsarrow{i}(2).YData = y2;
        h_carsarrow{i}(3).XData = x3; h_carsarrow{i}(3).YData = y3;
        
        if show_text
            h_texts{i}.Position = [othercars.car{i}.pos(1), othercars.car{i}.pos(2)];
        end
    end
    
    % Random paths
    for i = 1:othercars.n
        randpaths = othercars.car{i}.paths;
        npath = size(randpaths, 1)/3;
        if npath > 0
            cpaths = [];
            for j = 1:npath
                cpath = randpaths(3*j-2:3*j, :)';
                cpaths = [cpaths ; NaN NaN ; cpath(:, 1:2)];
            end
            h_paths{i}.XData = cpaths(:, 1);
            h_paths{i}.YData = cpaths(:, 2);
        end
    end
end



