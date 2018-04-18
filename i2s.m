function state = i2s(idx, opt)

switch opt.type
    case 'xyd'
        xmin = opt.xmin;
        xmax = opt.xmax;
        ymin = opt.ymin;
        ymax = opt.ymax;
        dmin = opt.dmin;
        dmax = opt.dmax;
        
        xres = opt.xres;
        yres = opt.yres;
        dres = opt.dres;
        
        xunit = (xmax - xmin)/(xres);
        yunit = (ymax - ymin)/(yres);
        dunit = (dmax - dmin)/(dres);
        
        quots = idx;
        didx = rem(quots,dres);
        quots = fix(quots/dres);
        if didx == 0
            quots = quots - 1;
            didx = dres;
        end
        
        xidx = fix(quots/yres)+1;
        yidx = rem(quots,yres)+1;
        
        x = xmin + xidx*xunit - xunit/2;
        y = ymin + yidx*yunit - yunit/2;
        d = dmin + didx*dunit - dunit/2;
        
        state = [x y d];
    case 'xy'
        xmin = opt.xmin;
        xmax = opt.xmax;
        ymin = opt.ymin;
        ymax = opt.ymax;
        
        xres = opt.xres;
        yres = opt.yres;
        
        xunit = (xmax - xmin)/xres;
        yunit = (ymax - ymin)/yres;
        
        quots = idx;
        xidx = fix(quots/yres)+1;
        yidx = rem(quots,yres);
        if yidx == 0
            xidx = xidx - 1;
            yidx = yres;
        end
        
        x = xmin + xidx*xunit - xunit/2;
        y = ymin + yidx*yunit - yunit/2;
        
        state = [x y];
end