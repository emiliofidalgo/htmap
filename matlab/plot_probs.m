function plot_probs(lik_file, pos_file, id, fin)

    lik = load(lik_file);
    pos = load(pos_file);    

    % Likelihood file.
    figure(1);
    clf;
    hold on;
    plot(lik(id,1:fin));
    xlabel('Frames');
    ylabel('Likelihood');
    m = mean(lik(id,:));
    s = std(lik(id,:));
    %plot([0,fin],[m,m],'r');
    %plot([0,fin],[m + 2*s,m + 2*s],'g--', 'LineWidth', 4.0);
    %set(gca, 'DataAspectRatio', [300 1 1])
    %ylim([0.99 1.05]);    
    hold off;
    
    % Posterior file    
    figure(2);
    clf;
    hold on;
    plot(pos(id,1:fin));
    xlabel('Frames');
    ylabel('Posterior');
    %set(gca, 'DataAspectRatio', [300 1 1])
    %ylim([0.0 0.1]);
    hold off;
    
end