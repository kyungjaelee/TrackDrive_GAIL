function out = get_nzval(nzr_x, val)
% GET NORMALIZED VALUE

n = size(val, 1);
out = (val - repmat(nzr_x.mu, n, 1)) ...
    ./ repmat(nzr_x.sig+nzr_x.eps, n, 1);
