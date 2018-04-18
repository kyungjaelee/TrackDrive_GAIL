function save_openfigures2pngs(folderpath, closeflag)

if nargin == 0
    folderpath = '';
    closeflag = 0;
elseif nargin == 1
    closeflag = 0;
end

h = get(0, 'children');
for i = 1:numel(h)
    ch = h(i);
    savename = [folderpath, '/', ch.Name, '.png'];
    set(ch,'PaperPositionMode','auto')
    print (ch , '-dpng', savename) ;
    fprintf('[%02d/%02d] %s saved. \n', i, numel(h), savename);
end

if closeflag
    % fprintf('Closing all. \n');
    close all;
end
