function [val, grad] = multiple_maxentopt(w, mdp, demo_info)
persistent mu_sa_hat init_s first_flag
if isempty(first_flag)
    first_flag = 1;
end
feat_dim = size(w, 1);
val = 0;
grad = 0;
% Multiple MDP setting
for mdp_idx = 1:demo_info.n
    % For each MDP
    mdp_model = mdp;
    mdp_model.feature = demo_info.F{mdp_idx};
    mdp_model.f_dim = feat_dim;
    
    if first_flag
        first_flag = 0;
        demonstration = demo_info.demos{mdp_idx};
        [ mu_sa_hat, init_s ] = datastatistics( mdp_model, demonstration );
    end
    
    [cval, cgrad] = maxentopt(w, mu_sa_hat, init_s, mdp_model);
    val = val + cval;
    grad = grad + cgrad;
end

% Average
val = val / demo_info.n;
grad = grad / demo_info.n;
