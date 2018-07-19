function plot_descinfo(directory)
    close all;

    files = dir(directory);
    
    totals = [];
    total_files = 0;
    
    min_av = 256;
    max_av = 0;
    max_av_data = [];
    min_av_data = [];
    max_std = 0;
    max_std_data = [];
    
    for i=1:length(files)
        if files(i).isdir
            continue
        end        
       
        total_files = total_files + 1;        
        file = strcat(directory, char(files(i).name));
        
        z = load(file);
        %imhist(uint8(z(:,1)));
        %title(file);
        m = mean(z(:,1));
        sd = std(z(:,1));                
        
        if m > max_av
            max_av = m;
            max_av_data = z(:,1);
        end
        
        if m < min_av
            min_av = m;
            min_av_data = z(:,1);
        end
        
        if sd > max_std
            max_std = sd;
            max_std_data = z(:, 1);
        end
        
        totals = [totals; [z(:, 1), z(:, 2)]];
        %pause;
    end
    
    tfontsize = 30;
    afontsize = 30;    
    
    % Min Histogram
    figure;
    hist(min_av_data(:,1), 256);
    %title('Min Avg Histogram', 'FontSize', tfontsize);
    xlim([0 255]);
    %set(gca,'YtickLabel',[]);
    xlabel('# of zeros in the descriptor', 'FontSize', afontsize);
    ylabel('# of descriptors', 'FontSize', afontsize);
    set(gca,'FontSize', 20);
    disp(['Mean (min): ', num2str(min_av)]);
    print -dpng -r300 descinfo_min
    
    % Max Histogram
    figure;
    hist(max_av_data(:,1), 256);
    %title('Max Avg Histogram', 'FontSize', tfontsize);
    xlim([0 255]);
    %set(gca,'YtickLabel',[]);
    xlabel('# of zeros in the descriptor', 'FontSize', afontsize);
    ylabel('# of descriptors', 'FontSize', afontsize);
    set(gca,'FontSize', 20);
    disp(['Mean (max): ', num2str(max_av)]);
    print -dpng -r300 descinfo_max
    
    % Max Std Histogram
    figure;
    hist(max_std_data(:,1), 256);
    %title('Max Std Histogram', 'FontSize', tfontsize);
    xlim([0 255]);
    %set(gca,'YtickLabel',[]);
    xlabel('# of zeros in the descriptor', 'FontSize', afontsize);
    ylabel('# of descriptors', 'FontSize', afontsize);
    set(gca,'FontSize', 20);
    disp(['Std (std): ', num2str(max_std)]);
    print -dpng -r300 descinfo_std
    
    % Total histogram
    figure
    [h, x] = hist(totals(:,1), 256);
    h = h/sum(h);
    bar(x, h);
    %title('Total Histogram', 'FontSize', tfontsize);
    xlim([0 255]);
    %set(gca,'YtickLabel',[]);
    xlabel('# of zeros in the descriptor', 'FontSize', afontsize);
    ylabel('# of descriptors', 'FontSize', afontsize);
    set(gca,'FontSize', 20);
    m = mean(totals(:,1));
    sd = std(totals(:,1));   
    
    hold on;
    x = [0:1:256];
    norm = normpdf(x, m, sd);
    plot(x, norm, 'r')
    hold off;
    
    tdesc = size(totals, 1);
    disp(['Mean (Total): ', num2str(m)]);
    print -dpng -r300 descinfo_total   
    
    disp(['------']);
    disp(['Total Number of Descriptors: ', num2str(tdesc)]);
    disp(['Total Nodes: ', num2str(total_files)]);    
end