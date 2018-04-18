function res ...
    = get_kde(data, x1min, x1max, x1res ...
    , x2min, x2max, x2res, hyp)

res.data = data;
res.axis = [x1min x1max x2min x2max];
res.n = size(data, 1);
res.x1grid = linspace(x1min, x1max, x1res);
res.x2grid = linspace(x2min, x2max, x2res);
[x1mesh, x2mesh] = meshgrid(res.x1grid, res.x2grid);
res.xpnts = [x1mesh(:) x2mesh(:)];
res.npnts = x1res*x2res;

res.K = kernel_se(res.xpnts, res.data(:, 1:2) ...
    , ones(res.npnts, 1), ones(res.n, 1), [hyp 1]);
res.z = sum(res.K, 2); res.z = res.z / sum(res.z);
res.Z = reshape(res.z, x2res, x1res);

