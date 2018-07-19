function print_likelihood(gtimage_path)
    % GT image
    figure;
    gtim = load(gtimage_path);
    imagesc(gtim.truth);
    colormap('pink');
    xlabel('Frame');
    ylabel('Frame');
    print -djpeg -r300 groundtruth
    
    % Likelihood
    figure;
    lik = load('results/Hamap/NewCollege/hamap_likelihood_125.txt');
    imagesc(lik);
    colormap('pink');
    xlabel('Frame');
    ylabel('Frame');
    print -djpeg -r300 likelihood
end