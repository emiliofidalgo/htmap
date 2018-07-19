function plot_topmap(tmap_file, coords_file)
    tmap = load(tmap_file);
    coords = load(coords_file);
        % For CC and NC
        %coords = coords.GPS(1:2:end, :);
        % For KITTIs
        c = coords.p;
        % For KITTI 06
        %c(:, 1) = coords.p(:, 2);
        %c(:, 2) = -coords.p(:, 1);
        % For KITTIs
        coords = c;       
        % For St. Lucia
        %coords = coords.fGPS(1:end, :);
    nimages = length(tmap);
    
    total_nodes = max(tmap(:,2)) + 1;
    
    %cmap = prism(total_nodes);
    cmap = rand(total_nodes, 3);
    
    figure()
    hold on;
    for i=2:nimages
        plot([coords(i - 1, 1), coords(i, 1)], [coords(i - 1, 2), coords(i, 2)], 'k-', 'LineWidth', 3, 'Color', cmap(tmap(i,2) + 1, :));
        %pause(0.005);
    end
    
    hold off
    axis off
    
    print -dpng -r300 image
    
    total_nodes
end
