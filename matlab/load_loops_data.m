function loops_mat = load_loops_data(loops_filepath)
    loops_file = fopen(loops_filepath);
    lines = {};
    while true
        line = fgetl(loops_file);
        if ~ischar( line ); break; end
        lines{end+1, 1} = line;
    end
    fclose(loops_file);

    loops_mat = zeros(size(lines, 1));
    for i=1:size(lines, 1)
        tokens = split(lines{i});
        for j=1:size(tokens)
            token = tokens{j};
            if isempty(token)
                continue
            else
                token = str2num(token) + 1;
                loops_mat(i, token) = 1;
            end
        end
    end
end
