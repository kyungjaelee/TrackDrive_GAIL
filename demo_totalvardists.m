ccc

%%

ndata1 = 100;
ndata2 = 200;
data1 = repmat([1 2], ndata1, 1) + randn(ndata1, 2);
data2 = repmat([2 1], ndata2, 1) + randn(ndata2, 2);

figure(1); hold on;
plot(data1(:, 1), data1(:, 2), 'ro');
plot(data2(:, 1), data2(:, 2), 'bx');
axis equal;

%%

x1min = -2; x1max = 5;
x2min = -2; x2max = 5;
x1res = 50; x2res = 50;
x1grid = linspace(x1min, x1max, x1res);
x2grid = linspace(x2min, x2max, x2res);
[x1mesh, x2mesh] = meshgrid(x1grid, x2grid);
xpnts = [x1mesh(:) x2mesh(:)];
npnts = x1res*x2res;
        
invlen = 1;
K1 = kernel_se(xpnts, data1(:, 1:2) ...
    , ones(npnts, 1), ones(ndata1, 1), [invlen invlen 1]);
z1 = sum(K1, 2); z1 = z1 / sum(z1);
Z1 = reshape(z1, x2res, x1res);
K2 = kernel_se(xpnts, data2(:, 1:2) ...
    , ones(npnts, 1), ones(ndata2, 1), [invlen invlen 1]);
z2 = sum(K2, 2); z2 = z2 / sum(z2);
Z2 = reshape(z2, x2res, x1res);

% Plot
fig = figure(2); clf;
subaxes(fig, 1, 2, 1, 0.15, 0.1); hold on;
pcolor(x1grid, x2grid, Z1);
plot(data1(:, 1), data1(:, 2), 'ro');
colormap gray; axis equal; axis([x1min x1max x2min x2max]);
title('Data1', 'FontSize', 15);
subaxes(fig, 1, 2, 2, 0.15, 0.1); hold on;
pcolor(x1grid, x2grid, Z2);
plot(data2(:, 1), data2(:, 2), 'bx');
colormap gray; axis equal; axis([x1min x1max x2min x2max]);
title('Data2', 'FontSize', 15);

%%

tvd = 0.5*sum(abs(z1-z2));
fprintf('totalVariationalDistance is %.3f \n', tvd)





