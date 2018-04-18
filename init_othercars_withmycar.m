function othercars = init_othercars_withmycar(track, mycar, nothercars, opt)

if nargin == 2
    opt = 'stop';
end

othercars = init_cars(5000, 3200);
setrange = [2 track.nr_seg]; lanerange = [1 track.nr_lane];
randposlist = [];
n = 0;
while n < nothercars
     randpos =  get_posintrack(track, randi(setrange), 0, randi(lanerange), 0);
     randposlist = [randposlist ; randpos];
     
     temp = randposlist - repmat(randpos, size(randposlist, 1), 1);
     temp2 = temp(:, 1)+temp(:, 2)+temp(:, 3);
     if length(find(abs(temp2) < 1E-5)) ~= 1, continue; end;
     temp = mycar.pos(1:2) - randpos(1:2);
     temp2 = norm(temp);
     if temp2 < 1000, continue; end;
     
     % Actual appending is done here,
     othercars = add_car(othercars, randpos, [0 0], opt);
     n = n + 1;
end