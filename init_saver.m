function saver = init_saver()

saver.n    = 0;
saver.pos  = [0 0 0];
saver.distth = 3000;
saver.degth  = 2;

saver.myinfo = cell(1E4, 1);
saver.mycar  = cell(1E4, 1);

saver.label  = 1; % 1: positive / 0: negative 
