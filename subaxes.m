function subaxes(figHandler, h, w, index, xmargin, ymargin)
% subplot? ???, margin? ?? ??? ? ??? ??? ??. 
% 
% figHandler : Figure handler
% h          : ?? subfigure? height
% w          : ?? subfigure? width
% index      : subfigure? inded
% (margin)   : margin
% 
if nargin == 4
    xmargin = 0.05;
    ymargin = 0.05;
end

w_idx = rem(index, w);
if w_idx == 0, w_idx = w; end;
w_idx = w_idx - 1;
h_idx = ceil(index/w) - 1;
h_idx = h - h_idx - 1;

w_unit = 1/w;
h_unit = 1/h;

x_start = w_unit*(w_idx) + xmargin/4;
y_start = h_unit*(h_idx) + ymargin/4;
x_len = w_unit-xmargin/2;
y_len = h_unit-ymargin/2;

axes('Parent', figHandler ...
    , 'Position' ...
    , [x_start, y_start, x_len, y_len] );      
