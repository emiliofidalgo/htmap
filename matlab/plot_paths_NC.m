gtruth = '/datasets/NewCollege/groundtruth.mat';
coords = '/datasets/NewCollege/ImageCollectionCoordinates.mat';

plot_path('dloopclosure/dloopdet_nc_loops_k6_5.txt', coords);
axis off
title('DLOOP (Recall = 41.65%)', 'FontSize', 15);
print -djpeg -r300 path_dloopclosure_nc

plot_path('FabMapV2/fabmap_nc_loops.txt', coords);
axis off
title('FABMAPV2 (Recall = 51.91%)', 'FontSize', 15);
print -djpeg -r300 path_fabmap_nc
 
plot_path('DIRD/DIRD_nc_loops.txt', coords);
axis off
title('DIRD (Recall = 45.04%)', 'FontSize', 15);
print -djpeg -r300 path_dird_nc
 
plot_path('rtabmap/rtabmap_loops_nc.txt', coords);
axis off
title('RTABMAP (Recall = 89.51%)', 'FontSize', 15);
print -djpeg -r300 path_rtabmap_nc
 
plot_path('BRIEFGist/BRIEFGist_loops_nc.txt', coords);
axis off
title('BRIEFGist (Recall = 28.57%)', 'FontSize', 15);
print -djpeg -r300 path_briefgist_nc
 
plot_path('WI-SURF/wisurf_loops_nc.txt', coords);
axis off
title('WI-SURF (Recall = 19.61%)', 'FontSize', 15);
print -djpeg -r300 path_wisurf_nc

plot_path('WI-SIFT/wisift_loops_nc.txt', coords);
axis off
title('WI-SIFT (Recall = 49.64%)', 'FontSize', 15);
print -djpeg -r300 path_wisift_nc

plot_path('binmap/binmap_nc_loops.txt', coords);
axis off
title('BINMAP (Recall = 53.27%)', 'FontSize', 15);
print -djpeg -r300 path_binmap_nc
