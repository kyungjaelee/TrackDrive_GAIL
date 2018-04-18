function saver = save_transfer(saver, mycar, myinfo, othercars)

% Collision check
mindist = inf;
for i = 1:othercars.n
    othercarpos = othercars.car{i}.pos;
    dist = norm(mycar.pos(1:2) - othercarpos(1:2));
    if dist < mindist
        mindist = dist;
    end
end
% fprintf('%.1f\n',min(myinfo.center_fb_dists)) 

if (mindist < mycar.H - 200) || (myinfo.center_fb_dists(2) < 4000)
    saver.curr_col = 1;
else
    saver.curr_col = 0;
end
if saver.curr_col == 1
    if saver.prev_col == 0
        saver.nr_col = saver.nr_col + 1;
        % fprintf(2, '%d Collision! \n', saver.nr_col);
    end
end
saver.prev_col = saver.curr_col;

% Lane change detection
if myinfo.lane_idx ~= saver.prev_lane
    saver.nr_lanechange = saver.nr_lanechange + 1;
    saver.prev_lane = myinfo.lane_idx;
end

% Other statistics 
if norm(saver.pos_prev(1:2) - mycar.pos(1:2)) > saver.pos_th
    saver.pos_prev = mycar.pos;
    saver.n = saver.n + 1;
    n = saver.n;
    
    % Statistics
    saver.cdist(n) = myinfo.center_fb_dists(1);
    saver.rdist(n) = myinfo.right_fb_dists(1);
    saver.ldist(n) = myinfo.left_fb_dists(1);
    saver.dir_vel(n) = mycar.vel(1);
    saver.ang_vel(n) = mycar.vel(2);
    saver.lane_dev(n) = myinfo.lane_dev;
    saver.lane_deg(n) = myinfo.deg;
    
    % Others
    saver.mycar{n} = mycar;
    saver.myinfo{n} = myinfo;
    saver.othercars{n} = othercars;
    % fprintf('%d saved. \n', n);
end