
% ADD IRL PACKAGES
addpath('../IRL_pack/minFunc');
addpath('../IRL_pack/mdp');
addpath('../IRL_pack/mmp');
addpath('../IRL_pack/maxent');
addpath('../IRL_pack/relent');
addpath('../IRL_pack/bmrl');
addpath('../IRL_pack/gpirl');

% ADD APPROPRIATE CVX TO THE PATH
com = computer;
if isequal(com(1:3), 'MAC')
    addpath('../IRL_pack/cvx_mac');
elseif isequal(com(1:3), 'GLN')
    addpath('../IRL_pack/cvx_linux');
else
    addpath('../IRL_pack/cvx_windows');
end

fprintf('PATHS ADDED \n');