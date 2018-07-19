function PR_batch(directory, gt_file, gt_neigh)
    files = get_files(directory, '.txt');
    Ps = [1.0];
    Rs = [0.0];
    max_pr = 0.0;
    max_re = 0.0;
    max_file = '';
%     PRE = [];
%     REC = [];
    TEP = {'Test'};
    for i=1:length(files)
        file = char(files(i));
        [P, R] = PR(file, gt_file, 1, gt_neigh);       
        Ps = [Ps, P];
        Rs = [Rs, R];
        
%         PRE = [PRE, P];
%         REC = [REC, R];        
        TEP{i + 1} = file;
        
        if P > max_pr || (P == max_pr && R > max_re)
            max_pr = P;
            max_re = R;
            max_file = file;
        end
    end
    
    [Rs, I] = sort(Rs);
    Ps_a = Ps;
    T_a = TEP;
    for i=1:numel(I)
        Ps_a(i) = Ps(I(i));
        T_a{i} = TEP{I(i)};
    end
    Ps = Ps_a;
    TEP = T_a;
    
    % Precision to copy to pr_curves
    for i=1:length(files)
        fprintf('%.4f, ', Ps(i));
    end    
    fprintf('\n');
    
    % Recall to copy to pr_curves
    for i=1:length(files)
        fprintf('%.4f, ', Rs(i));
    end
    fprintf('\n');
    
    for i=1:length(files)
        fprintf('Pr %f, Re %f, File: %s\n', Ps(i), Rs(i), TEP{i});
    end  
        
%     disp('--');
%     for i=1:length(PRE)
%         fprintf('Pr: %f, Re: %f, file: %s\n', PRE(i), REC(i), TEP{i});
%         %disp(['Pr: ',num2str(PRE(i)),', Re: ',num2str(REC(i)),' T: ',TEP{i}]);
%     end
%     disp('--');   

    fprintf('MAX Pr %f, Re %f, file: %s\n', max_pr, max_re, max_file);
    
    figure(1);
    plot(Rs, Ps, 'r');
    xlabel('Recall');
    ylabel('Precision');
    xlim([0.0 1.0]);
    ylim([0.0 1.0]);
end