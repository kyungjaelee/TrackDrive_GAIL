function plot_kderes(res, col)

pcolor(res.x1grid, res.x2grid, res.Z);
shading interp;
axis(res.axis);
plot(res.data(:, 1), res.data(:, 2), 'x' ...
    , 'MarkerSize', 15, 'Color', col ...
    , 'LineWidth', 2);
