function feat = get_feat(info, ctrl)

if nargin == 0
     feat = 6;
     return
end

lf_dist = info.nleft_fb_dists(1);
cf_dist = info.ncenter_fb_dists(1);
rf_dist = info.nright_fb_dists(1);

% Which featrues will we use? 
use_ctrl = 0;
if use_ctrl
     feat = [info.nlane_dev, info.ndeg, lf_dist, cf_dist, rf_dist, ctrl(1)/20000];
else
     feat = [info.nlane_dev, info.ndeg, lf_dist, cf_dist, rf_dist];
end
