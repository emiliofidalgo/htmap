function loops = generateGroundTruth(dist_matrix, max_dist, remove_prev)
    loops = (dist_matrix < max_dist);
    [r, c] = size(loops);
    for i=1:r
        if i > remove_prev
            loops(i, i-remove_prev:end) = false;            
        else
            loops(i, :) = false;
        end
    end
end