function [opt_vel, mycarpaths] = car_controller(carpos, algname ...
    , track, othercars, lreward)

switch algname
    case 'bmrl'
        if 0
            [mycarpaths, ctrls, N, Kres] = get_predefinedpaths(carpos);
            costs = zeros(N, 1);
            for j = 1:N % For all paths
                cpath = mycarpaths(3*j-2:3*j, :)';
                cctrl = ctrls(2*j-1:2*j, :)';
                cost = 0;
                for k = round(linspace(Kres/4, Kres/2, 3))
                    cpos  = cpath(k, :);
                    cu    = cctrl(k, :);
                    cinfo = get_trackinfo(track, cpos, othercars);
                    feat  = get_feat(cinfo, cu);
                    temp = bmrl_reward(lreward.alphathat ...
                        , lreward.hypOpt, lreward.Xu, lreward.Lu ...
                        , lreward.nz, feat);
                    % This is for handling last case
                    if k == Kres, temp = temp + temp*1; end;
                    % Sum costs
                    cost  = cost + temp;
                end
                costs(j) = cost;
            end
            % Controller
            ntemp = 0; sum_vel = [0 0]; sum_cost = 0;
            for ii = 1:N
                if isnan( costs(ii) ) == false
                    ntemp = ntemp + 1;
                    currweight = costs(ii);
                    sum_vel = sum_vel + currweight*ctrls(2*ii-1:2*ii, 1)';
                    sum_cost = sum_cost + currweight;
                end
            end
            opt_vel = sum_vel/sum_cost;
        else
            [mycarpaths, ctrls, N, Kres] = get_randpaths(carpos);
            costs = zeros(N, 1);
            for j = 1:N % For all paths
                cpath = mycarpaths(3*j-2:3*j, :)';
                cctrl = ctrls(2*j-1:2*j, :)';
                cost = 0;
                for k = round(linspace(Kres/4, Kres/2, 3))
                    cpos  = cpath(k, :);
                    cu    = cctrl(k, :);
                    cinfo = get_trackinfo(track, cpos, othercars);
                    feat  = get_feat(cinfo, cu);
                    temp = bmrl_reward(lreward.alphathat ...
                        , lreward.hypOpt, lreward.Xu, lreward.Lu ...
                        , lreward.nz, feat);
                    % This is for handling last case
                    if k == Kres, temp = temp + temp*1; end;
                    % Sum costs
                    cost  = cost + temp;
                end
                costs(j) = cost;
            end
            % Controller
            [~, optidx] = max(costs);
            opt_vel =  ctrls(2*optidx-1:2*optidx, 1)';
        end
    case 'gpirl'
        [mycarpaths, ctrls, N, Kres] = get_predefinedpaths_long(carpos);
        costs = zeros(N, 1);
        for j = 1:N % For all paths
            cpath = mycarpaths(3*j-2:3*j, :)';
            cctrl = ctrls(2*j-1:2*j, :)';
            cost = 0;
            for k = round(linspace(1, Kres, 3))
                cpos  = cpath(k, :);
                cu    = cctrl(k, :);
                cinfo = get_trackinfo(track, cpos, othercars);
                feat  = get_feat(cinfo, cu);
                temp = kernel_se(get_nzval(lreward.nz,feat), lreward.Xu ...
                    , 1, lreward.Lu, lreward.hyp)*lreward.alpha;
                % This is for handling last case
                if k == Kres, temp = temp + temp*1; end;
                % Sum costs
                cost  = cost + temp;
            end
            costs(j) = cost;
        end
        % Controller
        [~, optidx] = max(costs);
        opt_vel =  ctrls(2*optidx-1:2*optidx, 1)';
    case {'maxent', 'relent'}
        [mycarpaths, ctrls, N, Kres] = get_predefinedpaths_long(carpos);
        costs = zeros(N, 1);
        for j = 1:N % For all paths
            cpath = mycarpaths(3*j-2:3*j, :)';
            cctrl = ctrls(2*j-1:2*j, :)';
            cost = 0;
            for k = round(linspace(1, Kres, 3))
                cpos  = cpath(k, :);
                cu    = cctrl(k, :);
                cinfo = get_trackinfo(track, cpos, othercars);
                feat  = get_feat(cinfo, cu);
                temp  = feat*lreward.best_wopt;
                % This is for handling last case
                if k == Kres, temp = temp + temp*1; end;
                % Sum costs
                cost  = cost + temp;
            end
            costs(j) = cost;
        end
        % Controller
        [~, optidx] = max(costs);
        opt_vel =  ctrls(2*optidx-1:2*optidx, 1)';
end