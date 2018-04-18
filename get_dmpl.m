function dmpl = get_dmpl(algname,nData,rSeed,MED_TRICK,VERBOSE)
%
% Get DMPL object
%
rng(rSeed);
fdir = dir([algname '/drive_record*']);
nmat = length(fdir);
totalfeat = [];
totallev  = [];
for i = 1:nmat % For all mat file
    fname = fdir(i).name; 
    l = load([algname '/' fname]);
    n = l.saverlist.n;
    for j = 1:n % For each episode
        curr_saver = l.saverlist.savers{j};
        curr_othercars = l.saverlist.othercars{j};
        curr_saver_n = curr_saver.n;
        curr_label   = curr_saver.label; % <= Positive or negative
        currfeats = [];
        for k = 1:curr_saver_n
            curr_myinfo = curr_saver.myinfo{k};
            curr_mycar = curr_saver.mycar{k};
            currfeat = get_feat(curr_myinfo, curr_mycar.vel);
            if isnan(sum(currfeat)) == 0 % if value is NOT NaN
                currfeats = [ currfeats ; currfeat];
                if curr_label == 1
                    totallev  = [totallev ; 1];
                else
                    totallev  = [totallev ; -1];
                end
            end
        end
        % Accumulate all data
        totalfeat = [totalfeat ; currfeats];
    end
end
dim = size(totalfeat, 2);
ntotal = size(totalfeat, 1);
if VERBOSE
    fprintf('Total [%d] features. \n', ntotal);
    fprintf('Using [%d] demonstrations for training. \n', nData);
end
% Add some noise
totalfeat = totalfeat + 0.001*randn(size(totalfeat));
% Run DMPL
Xd = totalfeat;
Ld = totallev;
lambda = 1;
beta   = 1;

% Select subset
rIdx = randperm(ntotal);
sIdx = rIdx(1:nData);
Xd = Xd(sIdx,:);
Ld = Ld(sIdx,:);

% Median trick
if MED_TRICK
    Nd = size(Xd, 1);
    dismat = cell(1, dim);
    deddist = zeros(1, dim);
    for i = 1:dim
        dismat{i} = pdist(Xd(randsample(Nd, min(1000, Nd)), i));
        deddist(i) = median(dismat{i});
    end
    mininvlen = 5;
    maxinvlen = 10;
    hypInit = [min(maxinvlen, max(mininvlen, deddist)) 1]';
else
    hypInit = [10*ones(1, dim) 1]';
end
opt  = struct('Display', 'on', 'LS_init', 1, 'LS', 1, 'Method', 'lbfgs' ...
    , 'MaxFunEvals', 4000, 'MaxIter', 1, 'TolFun', 1E-4, 'TolX', 1E-4, 'verbose', 0 ...
    , 'IndSet', 'track'); % track
[alphathat, hypOpt, Xu, Lu, nz] = bmrl(Xd, Ld, lambda, beta, hypInit, opt);


dmpl.alphathat = alphathat;
dmpl.hypOpt = hypOpt;
dmpl.Xd = Xd;
dmpl.Ld = Ld;
dmpl.Xu = Xu;
dmpl.Lu = Lu;
dmpl.nz = nz;

