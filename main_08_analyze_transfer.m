addpaths
ccc
%%
ccc
algnamelist  = {'bmrl', 'maxent', 'gpirl', 'relent'};
modelist = {'safe_driving_mode', 'speedy_gonzales', 'tailgating_mode'};
statlist = {'Absolute average lane deviation', 'Absolute average lane degree', 'Average number of collision' ...
    , 'Average directional velocity', 'Average number of lane change' ...
    , 'vdist of CenterDist vs. LaneDev', 'vdist of CenterDist vs. LaneDeg'};
nr_alg  = length(algnamelist);
nr_mode = length(modelist);
nr_stat = length(statlist);
plot_flag = 0;

totalstats = cell(nr_alg, nr_mode, nr_stat);
for algnameidx = 1:length(algnamelist) % For each IRL algorithm
    algname = algnamelist{algnameidx};
    for modenameidx = 1:length(modelist) % For each driving mode
        modename = modelist{modenameidx};
        
        % --------------------------------------------------%
        % 1.    Check recorded stattistic as a reference 
        %       - This is free from 'algname'
        % --------------------------------------------------%
        fdir_record = dir([modename '/drive_record*']);
        nmat_record = length(fdir_record);
        nposrec = 0; MAXSAVE = 2E3; cpos_prev = [0 0 0];
        cdist_record    = zeros(MAXSAVE, 1);
        rdist_record    = zeros(MAXSAVE, 1);
        ldist_record    = zeros(MAXSAVE, 1);
        ang_vel_record  = zeros(MAXSAVE, 1);
        lane_dev_record = zeros(MAXSAVE, 1);
        lane_deg_record = zeros(MAXSAVE, 1);
        for matidx = 1:nmat_record % For each mat file
            loadname = [modename '/' fdir_record(matidx).name];
            lrecord = load(loadname);
            % Parse information
            track = lrecord.track;
            othercars = lrecord.othercars;
            saverlist_record = lrecord.saverlist;
            % Save records!
            for saveridx = 1:saverlist_record.n % For each starting lanes (1~3)
                saver = saverlist_record.savers{saveridx};
                for posidx = 1:saver.n % <= For each car position
                    cpos = saver.mycar{posidx}.pos; % <= My car position
                    cvel = saver.mycar{posidx}.vel;
                    cmyinfo = saver.myinfo{posidx};
                    if norm(cpos_prev(1:2) - cpos(1:2)) > 2000 && cpos(1) < track.xmax-5000
                        cpos_prev = cpos;
                        nposrec = nposrec + 1;
                        cdist_record(nposrec)    = cmyinfo.center_fb_dists(1);
                        rdist_record(nposrec)    = cmyinfo.right_fb_dists(1);
                        ldist_record(nposrec)    = cmyinfo.left_fb_dists(1);
                        ang_vel_record(nposrec)  = cvel(2);
                        lane_dev_record(nposrec) = cmyinfo.lane_dev;
                        lane_deg_record(nposrec) = cmyinfo.deg;
                    end
                end % for posidx = 1:saver.n % <= For each car position
            end % for saveridx = 1:saverlist_record.n % For each starting lanes (1~3)
        end % for matidx = 1:nmat_record % For each mat file
        cdist_record    = cdist_record(1:nposrec);
        rdist_record    = rdist_record(1:nposrec);
        ldist_record    = ldist_record(1:nposrec);
        ang_vel_record  = ang_vel_record(1:nposrec);
        lane_dev_record = lane_dev_record(1:nposrec);
        lane_deg_record = lane_deg_record(1:nposrec);
        
        % --------------------------------------------------%
        % 2.    Check statistics of transfered test
        % --------------------------------------------------% 
        fdir_transfer = dir([modename '/' algname '_transfer*']);
        nmat_transfer = length(fdir_transfer);
        cdist_transfer    = [];
        rdist_transfer    = [];
        ldist_transfer    = [];
        ang_vel_transfer  = [];
        dir_vel_transfer  = [];
        lane_dev_transfer = [];
        lane_deg_transfer = [];
        sum_col = 0; sum_nr_lanechange = 0;
        for matidx = 1:nmat_transfer % For each mat file
            loadname  = [modename '/' fdir_transfer(matidx).name];
            ltransfer = load(loadname);
            savertransfer     = ltransfer.saver;
            nr_transfer       = savertransfer.n;
            cdist_transfer    = [cdist_transfer ; savertransfer.cdist(1:nr_transfer)];
            rdist_transfer    = [rdist_transfer ;  savertransfer.rdist(1:nr_transfer)];
            ldist_transfer    = [ldist_transfer ;  savertransfer.ldist(1:nr_transfer)];
            ang_vel_transfer  = [ang_vel_transfer ; savertransfer.ang_vel(1:nr_transfer)];
            dir_vel_transfer  = [dir_vel_transfer ; savertransfer.mycar{1}.vel(1)];
            lane_dev_transfer = [lane_dev_transfer ; savertransfer.lane_dev(1:nr_transfer)];
            lane_deg_transfer = [lane_deg_transfer ; savertransfer.lane_deg(1:nr_transfer)];
            sum_col = sum_col + savertransfer.nr_col;
            sum_nr_lanechange = sum_nr_lanechange + savertransfer.nr_lanechange;
        end
        % Handling nan
        nanidx = find(isnan(lane_dev_transfer)==1 | isnan(lane_deg_transfer)==1);
        lane_dev_transfer(nanidx) = [];
        lane_deg_transfer(nanidx) = [];
        cdist_transfer(nanidx) = [];
        % Compute useful stats
        absavg_lane_dev = mean(abs(lane_dev_transfer)); % 1. Absolute average of lane_dev
        absavgg_lane_deg = mean(abs(lane_deg_transfer)); % 2. Absolute average of lane_deg
        avg_col = sum_col / nmat_transfer; % 3.  Average number of collision
        avg_dir_vel = mean(dir_vel_transfer); % 4. Average directional velocity 
        avg_nr_lanechange = sum_nr_lanechange / nmat_transfer; % 5. Average number of lane deviation
        
        % --------------------------------------------------%
        % 3.    Compute variational distance! 
        % --------------------------------------------------% 
        distmin = 5000; distmax = 45000; distres = 50;
        lanedevmin = -4000; lanedevmax = 4000; lanedevres = 50;
        lanedegmin = -60; lanedegmax = 60; lanedegres = 50;
        hypcdist = 1/4E6; hyplanedev = 1/1E5; hyplanedeg = 1/5E1;
        % 6. CenterDist vs. LaneDev
        kderes_cdistlanedev_rec = get_kde([cdist_record lane_dev_record] ...
            , distmin, distmax, distres, lanedevmin, lanedevmax, lanedevres, [hypcdist hyplanedev]);
        kderes_cdistlanedev_res = get_kde([cdist_transfer lane_dev_transfer] ...
            , distmin, distmax, distres, lanedevmin, lanedevmax, lanedevres, [hypcdist hyplanedev]);
        vdist_cdistlanedev = 0.5*sum(abs(kderes_cdistlanedev_rec.z - kderes_cdistlanedev_res.z));
        % 7. CenterDist vs. LaneDeg
        kderes_cdistlanedeg_rec = get_kde([cdist_record lane_deg_record] ...
            , distmin, distmax, distres, lanedegmin, lanedegmax, lanedegres, [hypcdist hyplanedeg]);
        kderes_cdistlanedeg_res = get_kde([cdist_transfer lane_deg_transfer] ...
            , distmin, distmax, distres, lanedegmin, lanedegmax, lanedegres, [hypcdist hyplanedeg]);
        vdist_cdistlanedeg = 0.5*sum(abs(kderes_cdistlanedeg_rec.z - kderes_cdistlanedeg_res.z));
        % Save 
        totalstats{algnameidx, modenameidx, 1} = absavg_lane_dev;
        totalstats{algnameidx, modenameidx, 2} = absavgg_lane_deg;
        totalstats{algnameidx, modenameidx, 3} = avg_col;
        totalstats{algnameidx, modenameidx, 4} = avg_dir_vel;
        totalstats{algnameidx, modenameidx, 5} = avg_nr_lanechange;
        totalstats{algnameidx, modenameidx, 6} = vdist_cdistlanedev;
        totalstats{algnameidx, modenameidx, 7} = vdist_cdistlanedeg;
        
        fprintf('[%d/%d][%d/%d] algname: %6s / modename: %18s / avg_col: %.2f \n' ...
            , algnameidx, length(algnamelist), modenameidx, length(modelist), algname, modename, avg_col);
        
        if plot_flag
            % 6. CenterDist vs. LaneDev
            fig = figure(2); clf;  set(fig, 'Position', [200 100+modenameidx*100 1000 500]);
            set(fig, 'Name', sprintf('fig_%s_%s_cdist_lanedev', algname, modename), 'NumberTitle', 'off');
            subaxes(fig, 1, 2, 1, 0.2, 0.25); hold on; colormap copper; caxis([0 0.0005]);
            plot_kderes(kderes_cdistlanedev_rec, 'g');
            xlabel('Center distance'); ylabel('Lane deviation');
            title(['['  modename '] Recorded demonstrations'], 'FontSize', 15, 'Interpreter', 'none');
            subaxes(fig, 1, 2, 2, 0.2, 0.25); hold on; colormap copper; caxis([0 0.0005]);
            plot_kderes(kderes_cdistlanedev_res, 'b');
            xlabel('Center distance'); ylabel('Lane deviation');
            title(['['  modename '] Resulting demonstrations'], 'FontSize', 15, 'Interpreter', 'none');
            
            % 7. CenterDist vs. LaneDeg
            fig = figure(3); clf;  set(fig, 'Position', [300 100+modenameidx*100 1000 500]);
            set(fig, 'Name', sprintf('fig_%s_%s_cdist_lanedeg', algname, modename), 'NumberTitle', 'off');
            subaxes(fig, 1, 2, 1, 0.2, 0.25); hold on; colormap copper; caxis([0 0.0005]);
            plot_kderes(kderes_cdistlanedeg_rec, 'g');
            xlabel('Center distance'); ylabel('Lane degree');
            title(['['  modename '] Recorded demonstrations'], 'FontSize', 15, 'Interpreter', 'none');
            subaxes(fig, 1, 2, 2, 0.2, 0.25); hold on; colormap copper; caxis([0 0.0005]);
            plot_kderes(kderes_cdistlanedeg_res, 'b');
            xlabel('Center distance'); ylabel('Lane degree');
            title(['['  modename '] Resulting demonstrations'], 'FontSize', 15, 'Interpreter', 'none');
            drawnow; pause;
        end
        
    end % for modenameidx = 1:length(modelist) % For each driving mode
end % for algnameidx = 1:length(algnamelist) % For each IRL algorithm
fprintf(2, 'Done. \n');

%%
clc
for i = 1:nr_mode
    curr_mode = modelist{i};
    fprintf('Current mode is [%s]. \n', curr_mode);
    fprintf('____________________________________________________________________________________________\n');
    fprintf('%35s', '')
    for j = 1:nr_alg
        curr_alg = algnamelist{j};
        fprintf('%12s', curr_alg)
    end
    fprintf('\n');
    for k = 1:nr_stat
        curr_stat = statlist{k};
        fprintf('%35s', curr_stat);
        for j = 1:nr_alg
            val = totalstats{j, i, k};
            fprintf('%12.3f', val);
        end
        fprintf('\n');
    end
    fprintf('\n');
end

%%




