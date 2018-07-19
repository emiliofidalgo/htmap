gtruth = '/datasets/CityCentre/groundtruth.mat';
coords = '/datasets/CityCentre/ImageCollectionCoordinates.mat';

plot_path('dloopclosure/dloopdet_cc_loops_k4_49.txt', coords);
axis off
title('DLOOP (Recall = 44.39%)', 'FontSize', 15);
print -djpeg -r300 path_dloopclosure_cc

plot_path('FabMapV2/fabmap_cc_loops.txt', coords);
axis off
title('FABMAPV2 (Recall = 38.50%)', 'FontSize', 15);
print -djpeg -r300 path_fabmap_cc
 
plot_path('DIRD/DIRD_cc_loops.txt', coords);
axis off
title('DIRD (Recall = 21.75%)', 'FontSize', 15);
print -djpeg -r300 path_dird_cc
 
plot_path('rtabmap/rtabmap_loops_cc.txt', coords);
axis off
title('RTABMAP (Recall = 77.54%)', 'FontSize', 15);
print -djpeg -r300 path_rtabmap_cc
 
plot_path('BRIEFGist/BRIEFGist_loops_cc.txt', coords);
axis off
title('BRIEFGist (Recall = 9.10%)', 'FontSize', 15);
print -djpeg -r300 path_briefgist_cc
 
plot_path('WI-SURF/wisurf_loops_cc.txt', coords);
axis off
title('WI-SURF (Recall = 4.28%)', 'FontSize', 15);
print -djpeg -r300 path_wisurf_cc

plot_path('WI-SIFT/wisift_loops_cc.txt', coords);
axis off
title('WI-SIFT (Recall = 60.10%)', 'FontSize', 15);
print -djpeg -r300 path_wisift_cc

plot_path('binmap/binmap_cc_loops.txt', coords);
axis off
title('BINMAP (Recall = 88.59%)', 'FontSize', 15);
print -djpeg -r300 path_binmap_cc
