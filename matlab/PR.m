function [precision, recall, results] = PR(loop_file, gt_file, load_file, gt_neigh)
    % Given a result loop matrix and a ground truth matrix, this function
    % computes the Precision/Recall values.
    
    if nargin < 4
        gt_neigh = 10;
    end
    
    % Loading loops and ground truth files.
    if load_file == 1
        loops = load(loop_file);
    else
        loops = loop_file;
    end
    gtruth = load(gt_file);
    
    % Defining general counters.
    TP = 0; % True Positives.
    FP = 0; % False Positives.
    TN = 0; % True Negatives.
    FN = 0; % False Negatives.
    
    loop_size = size(loops);
    gt_size = size(gtruth.truth);
    for i=1:loop_size(1)
        ones_index = find(loops(i, :));
        loop_closed = numel(ones_index) > 0;    % Is a loop detected ?
        loop_candidate = 0;                     % Which is the loop candidate ?
        gt_loop_closed = 0;                            % Is there a loop in the range indicated range according to the GT file ?.                            
        gt_nloops = numel(find(gtruth.truth(i, :)));  % Number of loops in the whole GT row.
        if loop_closed
            loop_candidate = ones_index(1);
            % Selecting the range to search in the ground truth file.
            ind1 = loop_candidate - gt_neigh;
            if ind1 < 1
                ind1 = 1;
            end
            ind2 = loop_candidate + (gt_neigh + 1);
            if ind2 > gt_size(2)
                ind2 = gt_size(2);
            end
            gt_value = gtruth.truth(i, ind1:ind2);    % Ground truth range around to the loop closure candidate.
            gt_loop_closed = numel(find(gt_value)) > 0;
        end
        
        % Taking a decision about this image.
        if loop_closed && gt_loop_closed
            TP = TP + 1;
            results(i) = 0;
        elseif loop_closed && (gt_nloops == 0 || ~gt_loop_closed)
            FP = FP + 1;
            results(i) = 1;
            sprintf('--------------False Positive! %d %d', i, ones_index)
        elseif ~loop_closed && gt_nloops == 0
            TN = TN + 1;
            results(i) = 2;
        elseif ~loop_closed && gt_nloops > 0
            FN = FN + 1;
            results(i) = 3;
        end       
        
        % Printing results to the command line.
        if loop_closed
            disp(['Image: ', int2str(i), ' Loop with: ', int2str(loop_candidate)]);
        else
            disp(['Image: ', int2str(i), ' No Loop detected']);
        end
    end
    
    % Printing final results.
    disp(['TP: ', int2str(TP)]);
    disp(['FP: ', int2str(FP)]);
    disp(['TN: ', int2str(TN)]);
    disp(['FN: ', int2str(FN)]);
    
    % Computing the Precision/Recall final values.
    precision = TP / (TP + FP);
    recall = TP / (TP + FN);
end