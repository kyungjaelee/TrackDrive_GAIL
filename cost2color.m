function colors = cost2color(costs)

n = size(costs, 1);
colors = jet(n);
[~, idx] = sort(costs);
colors = colors(idx, :);


infidx = find(costs == inf);
colors(infidx, :) = repmat([1 0 0], length(infidx), 1);