function plot_descinfo_full(dir1, dir2, dir3, dir4)
    close all;
    
    figure;    
    hold on;
    plot_total_hist(dir1, 'r*');
    plot_total_hist(dir2, 'g*');
    plot_total_hist(dir3, 'b*');
    plot_total_hist(dir4, 'k*');
    legend('25% (25 L)', '50% (74 L)', '75% (176 L)', '100% (258 L)', 'FontSize', 8, 'Location', 'NorthEast');
    hold off;
    print -dpng -r300 descinfo_full
end

function plot_total_hist(d, color)
    files = dir(d);
    
    totals = [];
    total_files = 0;
    
    for i=1:length(files)
        if files(i).isdir
            continue
        end
       
        total_files = total_files + 1;
        file = strcat(d, char(files(i).name));
        
        z = load(file);
        
        totals = [totals; [z(:, 1), z(:, 2)]];        
    end    
    
    afontsize = 30;    
    
    % Total histogram    
    [h, x] = hist(totals(:,1), 256);
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