function corrected = correct_fabmap_pdf(result_file, delete)
    mat_file = load(result_file);
    corrected = mat_file.psame;
    sz = size(corrected);
    for i=1:sz(1)
        if i <= delete
            corrected(i,:) = 0;
            continue;
        end
        corrected(i,i-delete:i) = 0;
        s = sum(corrected(i,:));
        if s > 0
            corrected(i, 1:i) = corrected(i, 1:i) ./ s;
        end
    end
end