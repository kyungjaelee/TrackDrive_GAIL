function modeidx = modename2idx(modename)

switch modename
    case 'safe_driving_mode', modeidx = 1;
    case 'speedy_gonzales', modeidx  = 2;
    case 'tailgating_mode', modeidx  = 3;
    case 'drunken_driving_mode', modeidx = 4;
    case 'mad_driving_mode', modeidx = 5;
end