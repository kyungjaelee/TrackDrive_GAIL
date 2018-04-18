function saver = init_transfersaver(track,pos_th)
% 1. cdist
% 2. rdist
% 3. ldist
% 4. ang_vel
% 5. lane_dev
% 6. lane_deg

if nargin == 1
    pos_th = 2000;
end

MAX_N = 5000;
saver.MAX_N = MAX_N;
saver.pos_prev = [0 0 0];
saver.pos_th = pos_th;
saver.n = 0;

% Things to save
saver.cdist = zeros(MAX_N, 1);
saver.rdist = zeros(MAX_N, 1);
saver.ldist = zeros(MAX_N, 1);
saver.dir_vel = zeros(MAX_N, 1);
saver.ang_vel = zeros(MAX_N, 1);
saver.lane_dev = zeros(MAX_N, 1);
saver.lane_deg = zeros(MAX_N, 1);

% Others
saver.track = track;
saver.mycar = cell(MAX_N, 1);
saver.myinfo = cell(MAX_N, 1);
saver.othercars = cell(MAX_N, 1);

% Collision check
saver.prev_col = 0;
saver.curr_col = 0;
saver.nr_col = 0;

% Lane dev
saver.prev_lane = inf;
saver.nr_lanechange = -1;

