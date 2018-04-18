addpaths
ccc
%% Part1: Analyze results
ccc
% =============================================================== %
algnamelist  = {'bmrl', 'maxent', 'gpirl', 'relent'};
% bmrl / maxent / gpirl / relent
modelist = {'safe_driving_mode', 'speedy_gonzales', 'tailgating_mode'};
% safe_driving_mode / speedy_gonzales / tailgating_mode
% drunken_driving_mode / mad_driving_mode
statlist = {'avg_colratio', 'vdist_xypos_avg', 'vdist_cdistavel' ...
    , 'vdist_cdistlanedev', 'vdist_cdistlanedeg', 'vdist_rdistavel', 'vdist_ldistavel'};
% =============================================================== %
nr_alg = length(algnamelist);
nr_mode = length(modelist);
nr_stat = length(statlist);

totalstats = cell(nr_alg, nr_mode, nr_stat);

plot_flag = false;
save_flag = false;

for algnameidx = 1:length(algnamelist) % For each IRL algorithm
    algname = algnamelist{algnameidx};
    modenameidx = 0;
    for modenameidx = 1:length(modelist) % For each driving mode
        modename = modelist{modenameidx};
        switch modename
            case 'safe_driving_mode', modeidx = 1;
            case 'speedy_gonzales', modeidx  = 2;
            case 'tailgating_mode', modeidx  = 3;
            case 'drunken_driving_mode', modeidx = 4;
            case 'mad_driving_mode', modeidx = 5;
        end
        fdir_record = dir([modename '/drive_record*']);
        fdir_result = dir([modename '/' algname '*']);
        nmat_record = length(fdir_record);
        nmat_result = length(fdir_result);
        
        poslist_record_all = []; poslist_result_all = [];
        devlist_record_all = []; devlist_result_all = [];
        deglist_record_all = []; deglist_result_all = [];
        dvellist_record_all = []; dvellist_result_all = [];
        avellist_record_all = []; avellist_result_all = [];
        cdist_record_all = []; cdist_result_all = [];
        rdist_record_all = []; rdist_result_all = [];
        ldist_record_all = []; ldist_result_all = [];
        vardist_xypos_sum = 0;
        
        fprintf('\n==================================================\n');
        fprintf('  Algorithm: %s Current mode: %s \n', algname, modename);
        fprintf('  Loading %d record and %d result mat file(s). \n', nmat_record, nmat_result);
        fprintf('==================================================\n');
        avg_col_list = zeros(nmat_record, 1);
        for i = 1:nmat_record % For each scenario (1~10) <= Other cars are fixed here
            % Load
            loadname = [modename '/' fdir_record(i).name];
            lrecord = load(loadname);
            loadname = [modename '/' fdir_result(i).name];
            lresult = load(loadname);
            % Parse
            track = lrecord.track;
            othercars = lrecord.othercars;
            saverlist_record = lrecord.saverlist;
            saverlist_result = lresult.saverlist;
            % Get other cars bds
            othercarbds = cell(othercars.n, 1);
            for ii = 1:othercars.n
                othercarbds{ii} = get_carshape(othercars.car{ii}.pos, othercars.car{ii}.W, othercars.car{ii}.H);
            end
            % Save records
            nposrec = 0; nposres = 0;
            poslist_record = zeros(2E3, 3); poslist_result = zeros(2E3, 3);
            devlist_record = zeros(2E3, 1); devlist_result = zeros(2E3, 1);
            deglist_record = zeros(2E3, 1); deglist_result = zeros(2E3, 1);
            dvellist_record = zeros(2E3, 1); dvellist_result = zeros(2E3, 1);
            avellist_record = zeros(2E3, 1); avellist_result = zeros(2E3, 1);
            cdist_record = zeros(2E3, 1); cdist_result = zeros(2E3, 1);
            rdist_record = zeros(2E3, 1); rdist_result = zeros(2E3, 1);
            ldist_record = zeros(2E3, 1); ldist_result = zeros(2E3, 1);
            
            cpos_prev = [0 0 0];
            for j = 1:saverlist_record.n % For each starting lanes (1~3)
                saver = saverlist_record.savers{j};
                othercars;
                for k = 1:saver.n % <= For each car position
                    cpos = saver.mycar{k}.pos; % <= My car position
                    cvel = saver.mycar{k}.vel;
                    cmyinfo = saver.myinfo{k};
                    % Density check
                    if norm(cpos_prev(1:2) - cpos(1:2)) > 2000 && cpos(1) < track.xmax-5000
                        cpos_prev = cpos;
                        nposrec = nposrec + 1;
                        poslist_record(nposrec, :) = cpos;
                        devlist_record(nposrec, :) = cmyinfo.lane_dev;
                        deglist_record(nposrec, :) = cmyinfo.deg;
                        dvellist_record(nposrec, :) = cvel(1);
                        avellist_record(nposrec, :) = cvel(2);
                        cdist_record(nposrec, :) = cmyinfo.center_fb_dists(1);
                        rdist_record(nposrec, :) = cmyinfo.right_fb_dists(1);
                        ldist_record(nposrec, :) = cmyinfo.left_fb_dists(1);
                    end
                end % for k = 1:saver.n % <= For each car position
            end
            poslist_record = poslist_record(1:nposrec, :);
            devlist_record = devlist_record(1:nposrec, :);
            deglist_record = deglist_record(1:nposrec, :);
            dvellist_record = dvellist_record(1:nposrec, :);
            avellist_record = avellist_record(1:nposrec, :);
            cdist_record = cdist_record(1:nposrec, :);
            rdist_record = rdist_record(1:nposrec, :);
            ldist_record = ldist_record(1:nposrec, :);
            % Save results
            cpos_prev = [0 0 0];
            nr_col = 0;
            for j = 1:saverlist_result.n % For each starting lanes (1~3)
                saver = saverlist_result.savers{j};
                iscol = 0;
                for k = 1:saver.n
                    cpos = saver.mycar{k}.pos;
                    cvel = saver.mycar{k}.vel;
                    cmyinfo = saver.myinfo{k};
                    % Collision check
                    mycarbd = get_carshape(cpos, saver.mycar{k}.W, saver.mycar{k}.H);
                    if iscol == 0
                        for kk = 1:othercars.n % For all other cars
                            cothercarbd = othercarbds{kk};
                            in = inpolygon(mycarbd(:, 1), mycarbd(:, 2), cothercarbd(:, 1), cothercarbd(:, 2));
                            if sum(in) > 0,  iscol = 1; break; end
                        end
                    end
                    if norm(cpos_prev(1:2) - cpos(1:2)) > 2000 && cpos(1) < track.xmax-5000
                        if isnan(cmyinfo.lane_dev) || isnan(cmyinfo.deg)
                            continue;
                        end
                        cpos_prev = cpos;
                        nposres = nposres + 1;
                        poslist_result(nposres, :) = cpos;
                        devlist_result(nposres, :) = cmyinfo.lane_dev;
                        deglist_result(nposres, :) = cmyinfo.deg;
                        dvellist_result(nposres, :) = cvel(1);
                        avellist_result(nposres, :) = cvel(2);
                        cdist_result(nposres, :) = cmyinfo.center_fb_dists(1);
                        rdist_result(nposres, :) = cmyinfo.right_fb_dists(1);
                        ldist_result(nposres, :) = cmyinfo.left_fb_dists(1);
                    end
                end
                if iscol
                    % fprintf(2, 'Collision occured. \n');
                    nr_col = nr_col + 1;
                end
            end % for j = 1:saverlist_result.n % For each starting lanes (1~3)
            avg_col = nr_col/saverlist_result.n; % <= This is the average collision ratio
            avg_col_list(i) = avg_col;
            
            poslist_result = poslist_result(1:nposres, :);
            devlist_result = devlist_result(1:nposres, :);
            deglist_result = deglist_result(1:nposres, :);
            dvellist_result = dvellist_result(1:nposres, :);
            avellist_result = avellist_result(1:nposres, :);
            cdist_result = cdist_result(1:nposres, :);
            rdist_result = rdist_result(1:nposres, :);
            ldist_result = ldist_result(1:nposres, :);
            % Now, we have all the data in 'saverlist_record' and 'saverlist_result'
            % First, compute variational distance between two empirical state distributions
            nr_lane = track.nr_lane;
            xlen = track.xmax - track.xmin;
            ylen = track.ymax - track.ymin;
            xres = 100;
            yres = nr_lane*6;
            xunit = xlen / xres;
            yunit = ylen / yres;
            xgrid = linspace(track.xmin, track.xmax, xres);
            ygrid = linspace(track.ymin, track.ymax, yres);
            [xmesh, ymesh] = meshgrid(xgrid, ygrid);
            xypnts = [xmesh(:) ymesh(:)];
            npnts = xres*yres;
            invlen = 5E-7;
            Krec = kernel_se(xypnts, poslist_record(:, 1:2) ...
                , ones(npnts, 1), ones(nposrec, 1), [invlen invlen 1]);
            zrec = sum(Krec, 2); zrec = zrec / sum(zrec);
            Zrec = reshape(zrec, yres, xres);
            Kres = kernel_se(xypnts, poslist_result(:, 1:2) ...
                , ones(npnts, 1), ones(nposres, 1), [invlen invlen 1]);
            zres = sum(Kres, 2); zres = zres / sum(zres);
            Zres = reshape(zres, yres, xres);
            vardist = 0.5*sum(abs(zrec-zres)); % <= Variational Distance
            vardist_xypos_sum = vardist_xypos_sum + vardist;
            % fprintf(' => VarDist is %.3f \n', vardist);
            % Secondly, we
            poslist_record_all = [poslist_record_all ; poslist_record];
            poslist_result_all = [poslist_result_all ; poslist_result];
            devlist_record_all = [devlist_record_all ; devlist_record];
            devlist_result_all = [devlist_result_all ; devlist_result];
            deglist_record_all = [deglist_record_all ; deglist_record];
            deglist_result_all = [deglist_result_all ; deglist_result];
            dvellist_record_all = [dvellist_record_all ; dvellist_record];
            dvellist_result_all = [dvellist_result_all ; dvellist_result];
            avellist_record_all = [avellist_record_all ; avellist_record];
            avellist_result_all = [avellist_result_all ; avellist_result];
            cdist_record_all = [cdist_record_all ; cdist_record];
            cdist_result_all = [cdist_result_all ; cdist_result];
            rdist_record_all = [rdist_record_all ; rdist_record];
            rdist_result_all = [rdist_result_all ; rdist_result];
            ldist_record_all = [ldist_record_all ; ldist_record];
            ldist_result_all = [ldist_result_all ; ldist_result];
            
            % Plot
            if plot_flag % Plot X-Y density
                fig = figure();
                set(fig, 'Position', [100 500 900 500]);
                set(fig, 'Name', sprintf('fig_%s_%s_xy_%02d', algname, modename, i), 'NumberTitle', 'off');
                clf;
                subaxes(fig, 2, 1, 1, 0.05, 0.1);
                hold on;
                pcolor(xgrid, ygrid, Zrec);
                clear plot_demo_othercars
                plot_demo_othercars(track, othercars);
                plot(poslist_record(:, 1), poslist_record(:, 2), 'ko', 'MarkerSize', 12, 'LineWidth', 2);
                % plot(poslist_result(:, 1), poslist_result(:, 2), 'w^', 'MarkerSize', 12, 'LineWidth', 2);
                title('Recorded Demonstrations', 'FontSize', 15);
                axis equal off;
                axis([track.xmin track.xmax track.ymin track.ymax])
                colormap jet; caxis([0 2E-3]);
                
                subaxes(fig, 2, 1, 2);
                hold on;
                pcolor(xgrid, ygrid, Zres);
                clear plot_demo_othercars
                plot_demo_othercars(track, othercars);
                % plot(poslist_record(:, 1), poslist_record(:, 2), 'ko', 'MarkerSize', 12, 'LineWidth', 2);
                plot(poslist_result(:, 1), poslist_result(:, 2), 'w^', 'MarkerSize', 12, 'LineWidth', 2);
                title('IRL Resulting Trajectories', 'FontSize', 15)
                axis equal off;
                axis([track.xmin track.xmax track.ymin track.ymax])
                colormap jet; caxis([0 2E-3]);
                drawnow;
            end
        end % for i = 1:nmat_result % For each scenario (1~10) <= Other cars are fixed here
        
        
        % Compute average vardist of xy poses
        vdist_xypos_avg = vardist_xypos_sum / nmat_result;
        % DO SOME ANALYSIS HERE!
        cdistmin = 5000; cdistmax = 45000; cdistres = 50;
        rdistmin = 5000; rdistmax = 45000; rdistres = 50;
        ldistmin = 5000; ldistmax = 45000; ldistres = 50;
        avelmin = -100; avelmax = 100; avelres = 50;
        lanedevmin = -4000; lanedevmax = 4000; lanedevres = 50;
        lanedegmin = -60; lanedegmax = 60; lanedegres = 50;
        
        % 0. Average collision ration
        avg_colratio = mean(avg_col_list);
        % 1. CenterDist + AngulrVel
        kderes_cdistavel_rec ...
            = get_kde([cdist_record_all avellist_record_all] ...
            , cdistmin, cdistmax, cdistres ...
            , avelmin, avelmax, avelres, [1/4E6 1/7E1]);
        kderes_cdistavel_res ...
            = get_kde([cdist_result_all avellist_result_all] ...
            , cdistmin, cdistmax, cdistres ...
            , avelmin, avelmax, avelres, [1/4E6 1/7E1]);
        % 2. CenterDist + laneDev
        kderes_cdistlanedev_rec ...
            = get_kde([cdist_record_all devlist_record_all] ...
            , cdistmin, cdistmax, cdistres ...
            , lanedevmin, lanedevmax, lanedevres, [1/4E6 1/1E5]);
        kderes_cdistlanedev_res ...
            = get_kde([cdist_result_all devlist_result_all] ...
            , cdistmin, cdistmax, cdistres ...
            , lanedevmin, lanedevmax, lanedevres, [1/4E6 1/1E5]);
        % 3. CenterDist + laneDeg
        kderes_cdistlanedeg_rec ...
            = get_kde([cdist_record_all deglist_record_all] ...
            , cdistmin, cdistmax, cdistres ...
            , lanedegmin, lanedegmax, lanedegres, [1/4E6 1/5E1]);
        kderes_cdistlanedeg_res ...
            = get_kde([cdist_result_all deglist_result_all] ...
            , cdistmin, cdistmax, cdistres ...
            , lanedegmin, lanedegmax, lanedegres, [1/4E6 1/5E1]);
        % 4. RightDist + AngularVel
        kderes_rdistavel_rec ...
            = get_kde([rdist_record_all avellist_record_all] ...
            , rdistmin, rdistmax, rdistres ...
            , avelmin, avelmax, avelres, [1/4E6 1/7E1]);
        kderes_rdistavel_res ...
            = get_kde([rdist_result_all avellist_result_all] ...
            , rdistmin, rdistmax, rdistres ...
            , avelmin, avelmax, avelres, [1/4E6 1/7E1]);
        % 5. LeftDist + AngularVel
        kderes_ldistavel_rec ...
            = get_kde([ldist_record_all avellist_record_all] ...
            , ldistmin, ldistmax, ldistres ...
            , avelmin, avelmax, avelres, [1/4E6 1/7E1]);
        kderes_ldistavel_res ...
            = get_kde([ldist_result_all avellist_result_all] ...
            , ldistmin, ldistmax, ldistres ...
            , avelmin, avelmax, avelres, [1/4E6 1/7E1]);
        % Compute total variational dists
        vdist_cdistavel = 0.5*sum(abs(kderes_cdistavel_rec.z - kderes_cdistavel_res.z));
        vdist_cdistlanedev = 0.5*sum(abs(kderes_cdistlanedev_rec.z - kderes_cdistlanedev_res.z));
        vdist_cdistlanedeg = 0.5*sum(abs(kderes_cdistlanedeg_rec.z - kderes_cdistlanedeg_res.z));
        vdist_rdistavel = 0.5*sum(abs(kderes_rdistavel_rec.z - kderes_rdistavel_res.z));
        vdist_ldistavel = 0.5*sum(abs(kderes_ldistavel_rec.z - kderes_ldistavel_res.z));
        
        fprintf(' AvgColRatio: %.3f \n', avg_colratio);
        fprintf(' X pos & Y pos: %.3f \n', vdist_xypos_avg);
        fprintf(' CenterDist & AngularVel: %.3f \n', vdist_cdistavel);
        fprintf(' CenterDist & LaneDev: %.3f \n', vdist_cdistlanedev);
        fprintf(' CenterDist & LaneDeg: %.3f \n', vdist_cdistlanedeg);
        fprintf(' RightDist & AngularVel: %.3f \n', vdist_rdistavel);
        fprintf(' LeftDist & AngularVel: %.3f \n', vdist_ldistavel);
        totalstats{algnameidx, modenameidx, 1} = avg_colratio;
        totalstats{algnameidx, modenameidx, 2} = vdist_xypos_avg;
        totalstats{algnameidx, modenameidx, 3} = vdist_cdistavel;
        totalstats{algnameidx, modenameidx, 4} = vdist_cdistlanedev;
        totalstats{algnameidx, modenameidx, 5} = vdist_cdistlanedeg;
        totalstats{algnameidx, modenameidx, 6} = vdist_rdistavel;
        totalstats{algnameidx, modenameidx, 7} = vdist_ldistavel;
        
        % Plot!
        if plot_flag
            fig = figure(); clf;  set(fig, 'Position', [100 100+modenameidx*100 1000 500]);
            set(fig, 'Name', sprintf('fig_%s_%s_cdist_avel', algname, modename), 'NumberTitle', 'off');
            subaxes(fig, 1, 2, 1, 0.2, 0.25); hold on; colormap copper; caxis([0 0.0005]);
            plot_kderes(kderes_cdistavel_rec, 'g');
            xlabel('Center distance'); ylabel('Angular velocity');
            title(['['  modename '] Recorded demonstrations'], 'FontSize', 15, 'Interpreter', 'none');
            subaxes(fig, 1, 2, 2, 0.2, 0.25); hold on; colormap copper; caxis([0 0.0005]);
            plot_kderes(kderes_cdistavel_res, 'b');
            xlabel('Center distance'); ylabel('Angular velocity');
            title(['[' modename '] Resulting demonstrations'], 'FontSize', 15, 'Interpreter', 'none');
            
            fig = figure(); clf;  set(fig, 'Position', [200 100+modenameidx*100 1000 500]);
            set(fig, 'Name', sprintf('fig_%s_%s_cdist_lanedev', algname, modename), 'NumberTitle', 'off');
            subaxes(fig, 1, 2, 1, 0.2, 0.25); hold on; colormap copper; caxis([0 0.0005]);
            plot_kderes(kderes_cdistlanedev_rec, 'g');
            xlabel('Center distance'); ylabel('Lane deviation');
            title(['['  modename '] Recorded demonstrations'], 'FontSize', 15, 'Interpreter', 'none');
            subaxes(fig, 1, 2, 2, 0.2, 0.25); hold on; colormap copper; caxis([0 0.0005]);
            plot_kderes(kderes_cdistlanedev_res, 'b');
            xlabel('Center distance'); ylabel('Lane deviation');
            title(['['  modename '] Resulting demonstrations'], 'FontSize', 15, 'Interpreter', 'none');
            
            fig = figure(); clf;  set(fig, 'Position', [300 100+modenameidx*100 1000 500]);
            set(fig, 'Name', sprintf('fig_%s_%s_cdist_lanedeg', algname, modename), 'NumberTitle', 'off');
            subaxes(fig, 1, 2, 1, 0.2, 0.25); hold on; colormap copper; caxis([0 0.0005]);
            plot_kderes(kderes_cdistlanedeg_rec, 'g');
            xlabel('Center distance'); ylabel('Lane degree');
            title(['['  modename '] Recorded demonstrations'], 'FontSize', 15, 'Interpreter', 'none');
            subaxes(fig, 1, 2, 2, 0.2, 0.25); hold on; colormap copper; caxis([0 0.0005]);
            plot_kderes(kderes_cdistlanedeg_res, 'b');
            xlabel('Center distance'); ylabel('Lane degree');
            title(['['  modename '] Resulting demonstrations'], 'FontSize', 15, 'Interpreter', 'none');
            
            fig = figure(); clf;  set(fig, 'Position', [400 100+modenameidx*100 1000 500]);
            set(fig, 'Name', sprintf('fig_%s_%s_rdist_avel', algname, modename), 'NumberTitle', 'off');
            subaxes(fig, 1, 2, 1, 0.2, 0.25); hold on; colormap copper; caxis([0 0.0005]);
            plot_kderes(kderes_rdistavel_rec, 'g');
            xlabel('Right distance'); ylabel('Angular velocity');
            title(['['  modename '] Recorded demonstrations'], 'FontSize', 15, 'Interpreter', 'none');
            subaxes(fig, 1, 2, 2, 0.2, 0.25); hold on; colormap copper; caxis([0 0.0005]);
            plot_kderes(kderes_rdistavel_res, 'b');
            xlabel('Right distance'); ylabel('Angular velocity');
            title(['['  modename '] Resulting demonstrations'], 'FontSize', 15, 'Interpreter', 'none');
            
            fig = figure(); clf;  set(fig, 'Position', [500 100+modenameidx*100 1000 500]);
            set(fig, 'Name', sprintf('fig_%s_%s_ldist_avel', algname, modename), 'NumberTitle', 'off');
            subaxes(fig, 1, 2, 1, 0.2, 0.25); hold on; colormap copper; caxis([0 0.0005]);
            plot_kderes(kderes_ldistavel_rec, 'g');
            xlabel('Left distance'); ylabel('Angular velocity');
            title(['['  modename '] Recorded demonstrations'], 'FontSize', 15, 'Interpreter', 'none');
            subaxes(fig, 1, 2, 2, 0.2, 0.25); hold on; colormap copper; caxis([0 0.0005]);
            plot_kderes(kderes_ldistavel_res, 'b');
            xlabel('Left distance'); ylabel('Angular velocity');
            title(['['  modename '] Resulting demonstrations'], 'FontSize', 15, 'Interpreter', 'none');
            drawnow;
            pause;
        end
        
        % save all and close
        if save_flag
            save_openfigures2pngs('_images', 1);
        end
    end % for modenameidx = 1:length(modelist) % For each driving mode
end  % For each IRL algorithm
fprintf(2, 'Done. \n');

%%
clc;
algnamelist  = {'bmrl', 'maxent', 'gpirl', 'relent'};
modelist = {'safe_driving_mode', 'speedy_gonzales', 'tailgating_mode'};
statlist = {'avg_colratio', 'vdist_xypos_avg', 'vdist_cdistavel' ...
    , 'vdist_cdistlanedev', 'vdist_cdistlanedeg', 'vdist_rdistavel', 'vdist_ldistavel'};
totalstats; % nr_alg, nr_mode, nr_stat
for i = 1:nr_mode
    curr_mode = modelist{i};
    fprintf('Current mode is [%s]. \n', curr_mode);
    fprintf('____________________________________________________________\n');
    fprintf('%20s', ' ')
    for j = 1:nr_alg
        curr_alg = algnamelist{j};
        fprintf('%10s', curr_alg)
    end
    fprintf('\n');
    for k = 1:nr_stat
        curr_stat = statlist{k};
        fprintf('%20s', curr_stat);
        for j = 1:nr_alg
            val = totalstats{j, i, k};
            if k == 1
                fprintf('%9.1f%%', val*100);
            else
                fprintf('%10.3f', val);
            end
        end
        fprintf('\n');
    end
    fprintf('%20s', 'average var.dist.');
    for j = 1:nr_alg
        fprintf('%10.3f', mean([totalstats{j, i, 2:end}]))
    end
    fprintf('\n\n');
end

%% Save all open figures to pngs!
if false
    h = get(0, 'children');
    for i = 1:numel(h)
        ch = h(i);
        savename = ['_images/', ch.Name, '.png'];
        set(ch,'PaperPositionMode','auto')
        print (ch , '-dpng', savename) ;
        fprintf('[%02d/%02d] %s saved. \n', i, numel(h), savename);
    end
end
fprintf(2, 'Done. \n');

%%
ccc
