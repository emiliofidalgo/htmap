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

function plot_times_bars(dtimes_file, lc_file, t, show_legend)

    % Function to plot PR curves.
    lab_size = 26;
    ax_size = 20;
    leg_size = 17;
    
    dtimes = load(dtimes_file);
    lc = load(lc_file);
    
    % Compute DTIMES averages
    fast = dtimes(:, 1) * 1000;
    ldb = dtimes(:, 2) * 1000;
    phog = dtimes(:, 3) * 1000;
    fast_t = [];
    ldb_t = []; 
    phog_t = [];
    [r, ~] = size(fast);
    for i=1:100:r
        % FAST
        if i + 100 < r
            subc = fast(i:i+100);
        else
            subc = fast(i:end);
        end        
        m = mean(subc);
        fast_t = [fast_t, m];
        
        % LDB
        if i + 100 < r
            subc = ldb(i:i+100);
        else
            subc = ldb(i:end);
        end        
        m = mean(subc);
        ldb_t = [ldb_t, m];
        
        % PHOG
        if i + 100 < r
            subc = phog(i:i+100);
        else
            subc = phog(i:end);
        end        
        m = mean(subc);
        phog_t = [phog_t, m];
    end
    
    % Compute LC averages    
    bpred = lc(:, 1) * 1000;
    bupd = lc(:, 2) * 1000;
    llc = lc(:, 3) * 1000;
    ilc = lc(:, 4) * 1000;
    epgeom = lc(:, 5) * 1000;    
    
    bpred_t = [];
    bupd_t = [];
    llc_t = [];
    ilc_t = [];
    epgeom_t = [];
    [r, ~] = size(bpred);
    for i=1:100:r
        % BPRED
        if i + 100 < r
            subc = bpred(i:i+100);
        else
            subc = bpred(i:end);
        end        
        m = mean(subc);
        bpred_t = [bpred_t, m];
        
        % BUPD
        if i + 100 < r
            subc = bupd(i:i+100);
        else
            subc = bupd(i:end);
        end        
        m = mean(subc);
        bupd_t = [bupd_t, m];
        
        % LLC
        if i + 100 < r
            subc = llc(i:i+100);
        else
            subc = llc(i:end);
        end        
        m = mean(subc);
        llc_t = [llc_t, m];
        
        % ILC
        if i + 100 < r
            subc = ilc(i:i+100);
        else
            subc = ilc(i:end);
        end        
        m = mean(subc);
        ilc_t = [ilc_t, m];
        
        % EP GEOM
        if i + 100 < r
            subc = epgeom(i:i+100);
        else
            subc = epgeom(i:end);
        end        
        m = mean(subc);
        epgeom_t = [epgeom_t, m];
    end
    
    % Adding differences in sizes
    nels = size(fast_t, 2) - size(bpred_t, 2);
    for i=1:nels
        bpred_t = [0.0, bpred_t];
        bupd_t = [0.0, bupd_t];
        llc_t = [0.0, llc_t];
        ilc_t = [0.0, ilc_t];
        epgeom_t = [0.0, epgeom_t];
    end    
    
    fast_t = fast_t';    
    ldb_t = ldb_t';
    phog_t = phog_t';
    bpred_t = bpred_t';
    bupd_t = bupd_t';
    llc_t = llc_t';
    ilc_t = ilc_t';
    epgeom_t = epgeom_t';
    
    figure;    
    set(gca, 'FontSize', ax_size); 
    bar((1:size(fast_t,1)), [fast_t ldb_t phog_t [bpred_t + bupd_t] llc_t ilc_t epgeom_t], 'stacked');    
    axis tight
    if show_legend
        l = legend('FAST detection', 'LDB description', 'PHOG description', 'Bayes (Predict + Update)', 'Location Likelihood', 'Image Likelihood', 'Epipolar Analysis', 'Location', 'Northwest');
        set(l, 'FontSize', leg_size);
    end
    title(t);
    
    xlabel('# of frame (10^2)');
    ylabel('Average Exec. Time (ms)');
    
    print -dpng -r300 exectimes
    
    % Computing global averages
    disp('----');
    
    total = 0.0;
    
    m = mean(fast);
    total = total + m;
    s = std(fast);
    disp(['Fast Detection: ', num2str(m), ' (', num2str(s), ')']);
    
    m = mean(ldb);
    total = total + m;
    s = std(ldb);
    disp(['LDB Description: ', num2str(m), ' (', num2str(s), ')']);
    
    m = mean(phog);
    total = total + m;
    s = std(phog);
    disp(['PHOG Description: ', num2str(m), ' (', num2str(s), ')']);
    
    m = mean(bpred);
    total = total + m;
    s = std(bpred);
    disp(['Bayes Predict: ', num2str(m), ' (', num2str(s), ')']);
    
    m = mean(bupd);
    total = total + m;
    s = std(bupd);
    disp(['Bayes Update: ', num2str(m), ' (', num2str(s), ')']);
    
    m = mean(llc);
    total = total + m;
    s = std(llc);
    disp(['Location Likelihood: ', num2str(m), ' (', num2str(s), ')']);
    
    m = mean(ilc);
    total = total + m;
    s = std(ilc);
    disp(['Image Likelihood: ', num2str(m), ' (', num2str(s), ')']);
    
    m = mean(epgeom);
    total = total + m;
    s = std(epgeom);
    disp(['Epipolar Analysis: ', num2str(m), ' (', num2str(s), ')']);
    
    disp('----');    
    disp(['Average Execution Time: ', num2str(total)]);
    disp('----');
end