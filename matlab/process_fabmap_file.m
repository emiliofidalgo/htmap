function process_fabmap_file(in_file, discard, gt_file, gt_neigh, loop_init, loop_end, loop_inc)
    Ps = [1.0];
    Rs = [0.0];
    max_pr = 0.0;
    max_re = 0.0;
    max_mat = 0.0;
    for i=loop_init:loop_inc:loop_end
        fabmap_corrected = correct_fabmap_pdf(in_file, discard);
        fabmap_loops = transform_fabmap(fabmap_corrected, i);
        [P, R] = PR(fabmap_loops, gt_file, 0, gt_neigh);        
        Ps = [Ps, P];
        Rs = [Rs, R];
        
        if P > max_pr
            max_pr = P;
            max_re = R;
            max_mat = fabmap_loops;
        end
    end;
    
    [Rs, I] = sort(Rs);
    Ps_a = Ps;
    for i=1:numel(I)
        Ps_a(i) = Ps(I(i));
    end
    Ps = Ps_a;
    
    % Precision to copy to pr_curves
    for i=1:length(Ps)
        fprintf('%.4f, ', Ps(i));
    end    
    fprintf('\n');
    
    % Recall to copy to pr_curves
    for i=1:length(Rs)
        fprintf('%.4f, ', Rs(i));
    end
    fprintf('\n');
    
    for i=1:length(Ps)
        fprintf('Pr %f, Re %f\n', Ps(i), Rs(i));
    end
    
    fprintf('MAX Pr %f, Re %f\n', max_pr, max_re);
    dlmwrite('fabmap_loops.txt', max_mat, 'delimiter', '\t');
    
    figure(1);
    plot(Rs, Ps, 'r');
    xlabel('Recall');
    ylabel('Precision');
    xlim([0.0 1.0]);
    ylim([0.0 1.0]);   
end