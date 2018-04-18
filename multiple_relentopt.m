function [ val, grad ] = multiple_relentopt( w, mu_sa_hats, base_mus, mdp, demo_info, params )

feat_dim = length(w);
val = 0;
grad = 0;
for mdp_idx = 1:demo_info.n
    mdp_model = mdp;
    mdp_model.feature = demo_info.F{mdp_idx};
    mdp_model.f_dim = feat_dim;
    [ cval, cgrad ] = relentopt( w, base_mus, mu_sa_hats{mdp_idx}, mdp_model, params );    
    val = val + cval;
    grad = grad + cgrad;
end
% Average
val = val / demo_info.n;
grad = grad / demo_info.n;

end

