function saverlist = init_saverlist(track)

% saverlist = struct('savers', cell(100, 1) ... % myinfo / mycar / label
%     , 'othercars', cell(100, 1) ...
%     , 'n', 0 ...
%     , 'track', track);

saverlist.savers    = cell(100, 1);
saverlist.othercars = cell(100, 1);
saverlist.n         = 0;
saverlist.track     = track;
