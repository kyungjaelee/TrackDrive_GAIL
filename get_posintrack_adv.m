function pos = get_posintrack_adv(track, pos_opt)

seg_idx        = pos_opt.seg_idx;     % SEGMENT INDEX 
lane_idx       = pos_opt.lane_idx;    % LANE INDEX
dist_offset    = pos_opt.dist_offset; % DISTANCE OFFSET
devdeg_offset  = pos_opt.devdeg_offset;
devdist_offset = pos_opt.devdist_offset;

if lane_idx > track.nr_lane
    lane_idx = track.nr_lane;
end
if lane_idx < 1
    lane_idx  = 1;
end


% Offset to Lane center 
cseg      = track.seg{seg_idx};
width     = cseg.width;
unitwidth = width / track.nr_lane;

% FIND CURRENT LANE'S DIST
switch cseg.type
    case 'straight'
        cdist = cseg.d;
    case 'right_turn'
        d = cseg.d + width/2;
        cdist = 1/1*pi*(d - (lane_idx-0.5)*unitwidth);
    case 'left_turn'
        d = cseg.d + width/2;
        cdist = 1/1*pi*(d - width + (lane_idx-0.5)*unitwidth);
end
if dist_offset > cdist
    dist_offset = dist_offset - cdist;
    seg_idx     = seg_idx+ 1;
    cseg        = track.seg{seg_idx};    
end


% ADD DIST OFFSET
pos = cseg.startpos;
switch cseg.type
    case 'straight'
        width = cseg.width;
        unitwidth = width / track.nr_lane;
        temp = width/2 - unitwidth*(lane_idx-1) - unitwidth/2; 
        lane_deg = pos(3); c = cos(lane_deg*pi/180); s = sin(lane_deg*pi/180);
        pos(1) = pos(1) - temp*s;
        pos(2) = pos(2) + temp*c;  
        c = cos(cseg.startpos(3)*pi/180);
        s = sin(cseg.startpos(3)*pi/180);
        dir_vector = [c s];
        pos(1:2) = pos(1:2) + dist_offset*dir_vector;
        
        % LANE DEVIATION DISTANCE OFFSET
        pos(1) = pos(1) + devdist_offset*sin(pos(3)*pi/180);
        pos(2) = pos(2) - devdist_offset*cos(pos(3)*pi/180);
        
        % DEGREE OFFSET
        pos(3) = pos(3) + devdeg_offset; 
        
    case 'right_turn'
        centerpos = cseg.centerpos;
        real_r = cseg.d + cseg.width/2 - unitwidth/2 - unitwidth*(lane_idx-1);
        
        % FIRST, CONVERT, DIST OFFSET INTO DEG OFFSET
        cvd_offset_deg = 360*dist_offset/(2*pi*real_r);
        
        % OFFSET DEG THRESHOLDING
        cvd_offset_deg = mod(cvd_offset_deg+180, 360) - 180;
%         if cvd_offset_deg > 85, cvd_offset_deg = 85;
%         elseif cvd_offset_deg < -85, cvd_offset_deg = -85; 
%         end
        
        start_deg = -270 + cseg.startpos(3);
        next_deg = start_deg - cvd_offset_deg;
        next_c = cos(next_deg*pi/180);
        next_s = sin(next_deg*pi/180);
        pos(1:2) = centerpos + real_r*[next_c next_s];
        
        % LANE DEVIATION DISTANCE OFFSET
        pos(1) = pos(1) + devdist_offset*sin((pos(3)-cvd_offset_deg)*pi/180);
        pos(2) = pos(2) - devdist_offset*cos((pos(3)-cvd_offset_deg)*pi/180);
        
        % DEGREE OFFSET
        pos(3) = cseg.startpos(3) - cvd_offset_deg + devdeg_offset; 
        
        
    case 'left_turn' 
        centerpos = cseg.centerpos;
        real_r = cseg.d - cseg.width/2 + unitwidth/2 + unitwidth*(lane_idx-1);
        cvd_offset_deg = 360*dist_offset/(2*pi*real_r);
        
        % OFFSET DEG THRESHOLDING
        cvd_offset_deg = mod(cvd_offset_deg+180, 360) - 180;
        if cvd_offset_deg > 85, cvd_offset_deg = 85;
        elseif cvd_offset_deg < -85, cvd_offset_deg = -85; 
        end
        
        start_deg = -90 + cseg.startpos(3);
        
        next_deg = start_deg + cvd_offset_deg;
        next_c = cos(next_deg*pi/180);
        next_s = sin(next_deg*pi/180);
        pos(1:2) = centerpos + real_r*[next_c next_s];
        
        % LANE DEVIATION DISTANCE OFFSET
        pos(1) = pos(1) + devdist_offset*sin((pos(3)+cvd_offset_deg)*pi/180);
        pos(2) = pos(2) - devdist_offset*cos((pos(3)+cvd_offset_deg)*pi/180);
        
        % DEGREE OFFSET
        pos(3) = cseg.startpos(3) + cvd_offset_deg + devdeg_offset;
end

% ADD A SMALL OFFSET TO X AXIS FOR ENSURING THE CAR TO BE INSIDE THE TRACK
eps = 1E-3;
pos(1) = pos(1) + eps;

% NORMALIZE DEGREE TO BE WITHIN -180 ~ +180
pos(3) = mod(pos(3)+180, 360) - 180;