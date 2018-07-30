%
% This file is part of htmap.
%
% Copyright (C) 2018 Emilio Garcia-Fidalgo <emilio.garcia@uib.es> (University of the Balearic Islands)
%
% htmap is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% htmap is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with htmap. If not, see <http://www.gnu.org/licenses/>.

function plot_times(dtimes_file, lc_file)

    % Function to plot PR curves.
    lab_size = 26;
    ax_size = 20;
    leg_size = 14;
    %line_size = 3.0;

    figure;
    hold on;
    set(gca, 'FontSize', ax_size); 
    xlabel('# Images', 'FontSize', lab_size);
    ylabel('Execution Time (ms)', 'FontSize', lab_size);    
    %xlim([0., 1.02]);
    %ylim([0., 1.02]);
    grid('off');    
    
    dtimes = load(dtimes_file);
    lc = load(lc_file);
    
    disp('----');    
    
%     % FAST Times
%     m = mean(dtimes(:, 1)*1000);
%     s = std(dtimes(:, 1)*1000);
%     mx = max(dtimes(:, 1)*1000);
%     mn = min(dtimes(:, 1)*1000);
%     disp(['FAST: ', num2str(m), ' ', num2str(s), ' ' num2str(mx), ' ' num2str(mn)]);
%     sz = size(dtimes(:, 1));
%     plot(1:1:sz(1), dtimes(:, 1) * 1000, 'r');
%     
%     % LDB Times
%     m = mean(dtimes(:, 2)*1000);
%     s = std(dtimes(:, 2)*1000);
%     mx = max(dtimes(:, 2)*1000);
%     mn = min(dtimes(:, 2)*1000);   
%     disp(['LDB: ', num2str(m), ' ', num2str(s), ' ' num2str(mx), ' ' num2str(mn)]);
%     sz = size(dtimes(:, 2));
%     plot(1:1:sz(1), dtimes(:, 2) * 1000, 'b');
%     
%     % PHOG Times
%     m = mean(dtimes(:, 3)*1000);
%     s = std(dtimes(:, 3)*1000);
%     mx = max(dtimes(:, 3)*1000);
%     mn = min(dtimes(:, 3)*1000);    
%     disp(['PHOG: ', num2str(m), ' ', num2str(s), ' ' num2str(mx), ' ' num2str(mn)]);
%     sz = size(dtimes(:, 3));
%     plot(1:1:sz(1), dtimes(:, 3) * 1000, 'g');
%     
%     title('Description Times');    
%     l = legend('FAST detection', 'LDB description', 'PHOG description', 'Location', 'NorthEast');
%     set(l, 'FontSize', leg_size);
%     hold off;
%     print -djpeg -r300 exectimes_desc
    
    figure;
    hold on;
    set(gca, 'FontSize', ax_size); 
    xlabel('# Images', 'FontSize', lab_size);
    ylabel('Execution Time (ms)', 'FontSize', lab_size);    
    %xlim([0., 1.02]);
    %ylim([0., 1.02]);
    grid('off'); 
    
    % Bayes Predict Times
    m = mean(lc(:, 1) * 1000);
    s = std(lc(:, 1)* 1000);
    mx = max(lc(:, 1)* 1000);
    mn = min(lc(:, 1)* 1000);    
    disp(['Bayes Predict: ', num2str(m), ' ', num2str(s), ' ' num2str(mx), ' ' num2str(mn)]);
    
    % Bayes Update Times
    m = mean(lc(:, 2)* 1000);
    s = std(lc(:, 2)* 1000);
    mx = max(lc(:, 2)* 1000);
    mn = min(lc(:, 2)* 1000);    
    disp(['Bayes Update: ', num2str(m), ' ', num2str(s), ' ' num2str(mx), ' ' num2str(mn)]);    
    
    % LLC
    m = mean(lc(:, 3)* 1000);
    s = std(lc(:, 3)* 1000);
    mx = max(lc(:, 3)* 1000);
    mn = min(lc(:, 3)* 1000);    
    disp(['LLC: ', num2str(m), ' ', num2str(s), ' ' num2str(mx), ' ' num2str(mn)]);
    sz = size(lc(:, 3));
    plot(1:1:sz(1), lc(:, 3) * 1000, 'r');
    
    % ILC
    m = mean(lc(:, 4)* 1000);
    s = std(lc(:, 4)* 1000);
    mx = max(lc(:, 4)* 1000);
    mn = min(lc(:, 4)* 1000);    
    disp(['ILC: ', num2str(m), ' ', num2str(s), ' ' num2str(mx), ' ' num2str(mn)]);
    sz = size(lc(:, 4));
    plot(1:1:sz(1), lc(:, 4) * 1000, 'b');
    
    % Epipolar Geom
    m = mean(lc(:, 5)* 1000);
    s = std(lc(:, 5)* 1000);
    mx = max(lc(:, 5)* 1000);
    mn = min(lc(:, 5)* 1000);    
    disp(['Ep. Geometry: ', num2str(m), ' ', num2str(s), ' ' num2str(mx), ' ' num2str(mn)]);
    sz = size(lc(:, 5));
    plot(1:1:sz(1), lc(:, 5) * 1000, 'g');    
    
    sz = size(lc(:, 2));
    plot(1:1:sz(1), (lc(:, 1) + lc(:, 2)) * 1000, 'k');
    
    title('Loop Closure Times');    
    l = legend('Location Likelihood', 'Image Likelihood', 'Epipolar Analysis', 'Bayes (Predict + Update)', 'Location', 'NorthWest');
    set(l, 'FontSize', leg_size);
    hold off;
    print -djpeg -r300 exectimes_lc
    
    disp('----');    
end