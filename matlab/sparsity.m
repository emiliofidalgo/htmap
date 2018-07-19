function sparsity
    % Function to plot sparsity plots.
    lab_size = 26;
    ax_size = 20;
    leg_size = 20;
    line_size = 3.0;
    
    %CC.nodes = [317,    253,    100,    63,     39,     19,     14,     0];
    %CC.recall = [0.08,  0.11,   0.32,   0.34,   0.36,   0.35,   0.36,   0];
    CC.nodes =  [310,   253,  143,  100,    63,     45,     39,     28,     24,     18,     0];
    CC.recall = [0.45,  0.52, 0.61, 0.67,   0.68,   0.69,   0.71,   0.713,  0.724,  0.7325, 0];
    NC.nodes  = [234,   146,    104,    74,     56,     33,     24,     18,     10,     9,      7,      0];
    NC.recall = [0.48,  0.55,   0.62,   0.65,   0.64,   0.65    0.65,   0.65,   0.68,   0.54,   0.47,   0];
   
    figure();
    hold on;    
    plot_sparsity(CC.recall, CC.nodes, 'b');
    plot_sparsity(NC.recall, NC.nodes, 'r');
    plot_common();
    l = legend('City Center', 'New College', 'Location', 'SouthEast');
    set(l, 'FontSize', leg_size)
    %title('Sparsity');
    
    print -dpng -r300 sparsity
    hold off; 
    
    function plot_common()         
        xlabel('# Locations', 'FontSize', lab_size);
        ylabel('Recall', 'FontSize', lab_size);        
        %xlim([0., 100]);
        ylim([0., 1.02]);
        grid('off');
        set(gca, 'FontSize', ax_size);
    end

    function plot_sparsity(pr, re, format)
        plot(re, pr, 'Color', format, 'LineWidth', line_size);
    end
end
