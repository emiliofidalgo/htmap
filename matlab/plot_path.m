function plot_path(loops_file, coords_file)
    loops = load(loops_file);
    %loops = loops.truth;
    coords = load(coords_file);
    % For CC and NC
    coords = coords.GPS(1:2:end, :);
        % For KITTIS
        %c = coords.p;
        % For KITTI 06
        %c(:, 1) = coords.p(:, 2);
        %c(:, 2) = -coords.p(:, 1);
        % For KITTIs
        %coords = c;        
        % For St. Lucia
        %coords = coords.fGPS(1:end, :);
    nimages = length(loops);
    figure();
    hold on;
    axis equal
    for i=2:nimages        
        [loops_index] = find(loops(i, :) > 0);
        if numel(loops_index > 0)% && results(i) ~= 1           
            plot([coords(loops_index(1), 1), coords(i, 1)], [coords(loops_index(1), 2), coords(i, 2)], 'g-', 'LineWidth', 1);
            plot([coords(i - 1, 1), coords(i, 1)], [coords(i - 1, 2), coords(i, 2)], 'r-', 'LineWidth', 3);            
            %plot(coords(loops_index(1), 1), coords(loops_index(1), 2), 'r-', 'LineWidth', 2);
            plot([coords(max(loops_index(1) - 1, 1), 1), coords(loops_index(1), 1)], [coords(max(loops_index(1) - 1, 1), 2), coords(loops_index(1), 2)], 'r-', 'LineWidth', 3);
        else
            plot([coords(i - 1, 1), coords(i, 1)], [coords(i - 1, 2), coords(i, 2)], 'k-', 'LineWidth', 3);
        end
        %pause(0.005);
    end    
    hold off;
    axis off;
    
    print -dpng -r300 image
end
