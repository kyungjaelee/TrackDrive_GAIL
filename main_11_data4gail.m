addpaths;
ccc;
%
% This code is for training policies
%
%% 1. BMRL (Proposed method)
ccc
algname = 'speedy_gonzales';
% safe_driving_mode / speedy_gonzales / tailgating_mode
% drunken_driving_mode / mad_driving_mode
fdir = dir([algname '/drive_record*']);
nmat = length(fdir);
fprintf('Loading %d mat file(s). \n', nmat);
totalfeat = [];
totalctrl  = [];
for i = 1:nmat % For all mat file
    fname = fdir(i).name;
    l = load([algname '/' fname]);
    n = l.saverlist.n;
    for j = 1:n % For each episode
        curr_saver = l.saverlist.savers{j};
        curr_othercars = l.saverlist.othercars{j};
        curr_saver_n = curr_saver.n;
        currfeats = [];
        currctrl = [];
        for k = 1:curr_saver_n
            curr_myinfo = curr_saver.myinfo{k};
            curr_mycar = curr_saver.mycar{k}
            currfeat = get_feat(curr_myinfo, curr_mycar.vel); % USE CONTROL
            if isnan(sum(currfeat)) == 0 % if value is NOT NaN
                currfeats = [ currfeats ; currfeat];
                currctrl = [currctrl ; curr_mycar.vel];
            end
        end
        % Accumulate all data
        totalfeat = [totalfeat ; currfeats];
        totalctrl = [totalctrl ; currctrl(:,1)];
    end
end
dim = size(totalfeat, 2);
ntotal = size(totalfeat, 1);
fprintf('Total %d features. \n', ntotal);

% Run BMRL
Xd = totalfeat;
Yd = totalctrl;

mu_Xd = mean(Xd);
std_Xd = std(Xd);
nz_Xd = (Xd - repmat(mu_Xd,ntotal,1))./repmat(std_Xd,ntotal,1);
mu_Yd = mean(Yd);
std_Yd = std(Yd);
nz_Yd = (Yd - repmat(mu_Yd,ntotal,1))./repmat(std_Yd,ntotal,1);


% Save
save_name = 'track_data.mat';
save(save_name, 'Xd', 'Yd', 'mu_Xd', 'std_Xd', 'nz_Xd', 'mu_Yd', 'std_Yd', 'nz_Yd');
fprintf(2, 'Track data are saved in %s \n', save_name);
