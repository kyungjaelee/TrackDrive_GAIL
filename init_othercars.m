function othercars = init_othercars(track, nothercars, opt)

if nargin == 2
    opt = 'stop';
end

othercars = init_cars(4500, 3000);
setrange = [2 track.nr_seg]; lanerange = [1 track.nr_lane];
randposlist = [];
n = 0; nRetry = 0;
while n < nothercars
     randpos =  get_posintrack(track, randi(setrange), 0, randi(lanerange), 0);
     randposlist = [randposlist ; randpos];
     temp = randposlist - repmat(randpos, size(randposlist, 1), 1);
     temp2 = sqrt(temp(:, 1).^2 + temp(:, 2).^2);
     if length(find(abs(temp2) < 1E4)) ~= 1
         nRetry = nRetry + 1;
         if nRetry > 1000 
             othercars = init_cars(4500, 3000);
             n = 0;
         end
         continue; 
     end
     
     % Actual appending is done here,
     othercars = add_car(othercars, randpos, [0 0], opt);
     n = n + 1;
end
