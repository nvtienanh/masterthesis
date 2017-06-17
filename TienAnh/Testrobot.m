% vectors I want to plot as rows (XSTART, YSTART, ZSTART) (XDIR, YDIR, ZDIR)
rays = [
  1 2 0  1 0 0;
  3 3 2  0 1 -1 ;
  0 1 -1  2 0 8;
  2 0 0  0 2 1;
] ;

% quiver plot
quiver3( rays( :,1 ), rays( :,2 ), rays( :,3 ), rays( :,4 ), rays( :,5 ), rays( :,6 ) );