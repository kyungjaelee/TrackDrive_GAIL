function [othercars, emsec] = ctrl_cars(othercars, track)
iclk = clock;

for i = 1:othercars.n
     ctrlmode = othercars.car{i}.ctrlmode;
     carpos   = othercars.car{i}.pos;
     switch ctrlmode
          case 'stop'
               
          case 'normal'
               forv = 7000; % <= Forward veloicty [mm/s] -> 25 km/h
               % Random paths
               N = 10; K = 45; T = 0.01; wmax = 300;
               iclk1 = clock;
               % rng(1)
               [paths, ctrls] = get_othercarpaths(carpos, N, K, [forv/2 forv], [-wmax wmax], T);
               emsec1 = etime(clock, iclk1)*1000;
               
               % Compute cost of each path
               iclk2 = clock;
               costs = zeros(N, 1);
               for j = 1:N % For all paths
                    cpath = paths(3*j-2:3*j, :)';
                    
                    % Initial Info
                    iinfo = get_trackinfo(track, cpath(1, :));
                    
                    % Path cost
                    cost = 0;
                    for k = K:K
                         cpos  = cpath(k, :);
                         cinfo = get_trackinfo(track, cpos);
                         cost  = cost + 1*cinfo.nlane_dev^2 + 1*cinfo.ndeg^2 + 1000*(iinfo.lane_idx-cinfo.lane_idx)^2;
                    end
                    costs(j) = cost;
               end
               [~, minidx] = min(costs);
               optpath = paths(3*minidx-2:3*minidx, :);
               othercars.car{i}.paths = optpath; % paths: all paths / optpath: optimal path / []: none
               emsec2 = etime(clock, iclk2)*1000;
               
               % Velocity
               othercars.car{i}.vel = ctrls(2*minidx-1:2*minidx, 1)';
               
               
     end
end

emsec = etime(clock, iclk)*1000;
% fprintf('%.1f / %.1f / %.1f \n', emsec1, emsec2, emsec);

