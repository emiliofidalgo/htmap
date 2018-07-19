function plot_descinfo_node(dir1, dir2, dir3, dir4, node)
    close all;
    
    figure;    
    hold on;
    plot_node_hist(dir1, node, 'r*');
    plot_node_hist(dir2, node, 'g*');
    plot_node_hist(dir3, node, 'b*');
    plot_node_hist(dir4, node, 'k*');
    legend('25%', '50%', '75%', '100%');
    hold off;
    
    print -dpng -r300 descinfo_node
end

function plot_node_hist(d, node, color)

    filename = sprintf('%d.txt', node);
    file = strcat(d, filename);
    z = load(file);
    
    afontsize = 30;    
    
    % Total histogram    
    [h, x] = hist(z(:,1), 256);
    h = h/sum(h);
    
    h1 = [];
    x1 = [];
    for i=1:length(h)
        if h(i) > 0
            h1 = [h1, h(i)];
            x1 = [x1, x(i)];
        end
    end
    plot(x1, h1, color);    
    xlim([0 255]);    
    xlabel('# of zeros in the descriptor', 'FontSize', afontsize);
    ylabel('# of descriptors', 'FontSize', afontsize);
    set(gca,'FontSize', 20);
end