function idx = s2i(state, opt)

switch opt.type
    case 'xyd'
        % State consists of (X, Y, and Degree)
        xmin = opt.xmin;
        xmax = opt.xmax;
        ymin = opt.ymin;
        ymax = opt.ymax;
        dmin = opt.dmin;
        dmax = opt.dmin;
        
        xres = opt.xres;
        yres = opt.yres;
        dres = opt.dres;
        
        xunit = (xmax - xmin)/(xres);
        yunit = (ymax - ymin)/(yres);
        dunit = (dmax - dmin)/(dres);
        
        x = state(1);
        y = state(2);
        d = state(3);
        
        xidx = min(xres, max(1, ceil((x - xmin)/xunit)));
        yidx = min(yres, max(1, ceil((y - ymin)/yunit)));
        didx = min(dres, max(1, ceil((d - dmin)/dunit)));
        
        idx = dres*yres*(xidx-1) + dres*(yidx-1) + didx;
    case 'xy'
        % State consists of (X, Y, and Degree)
        xmin = opt.xmin;
        xmax = opt.xmax;
        ymin = opt.ymin;
        ymax = opt.ymax;
        
        xres = opt.xres;
        yres = opt.yres;
        
        xunit = (xmax - xmin)/xres;
        yunit = (ymax - ymin)/yres;
        
        x = state(1);
        y = state(2);
        
        xidx = min(xres, max(1, ceil((x - xmin)/xunit)));
        yidx = min(yres, max(1, ceil((y - ymin)/yunit)));
        
        idx = yres*(xidx-1) + yidx;
end