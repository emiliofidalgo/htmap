function result = transform_fabmap(corrected_file, loop_thresh)
    %fabmap = load(fabmap_file);
    %nelems = length(fabmap.psame);
    fabmap = corrected_file;
    nelems = length(fabmap);
    result = false(nelems);
    for i=1:nelems
        [max_value, index] = max(fabmap(i, :));
        if max_value > loop_thresh
            result(i, index) = 1;            
        end
    end
end