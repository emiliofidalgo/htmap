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

function PR_curve(directory, gt_file, gt_neigh)
    files = get_files(directory, '.txt');
    Ps = [1.0];
    Rs = [0.0];
    max_pr = 0.0;
    max_re = 0.0;
    max_file = '';
%     PRE = [];
%     REC = [];
    TEP = {'None'};
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
    
    disp(['Final results']);

    for i=1:length(files)
        fprintf('Pr %f, Re %f, File: %s\n', Ps(i), Rs(i), TEP{i});
    end

%     disp('--');
%     for i=1:length(PRE)
%         fprintf('Pr: %f, Re: %f, file: %s\n', PRE(i), REC(i), TEP{i});
%         %disp(['Pr: ',num2str(PRE(i)),', Re: ',num2str(REC(i)),' T: ',TEP{i}]);
%     end
%     disp('--');

    fprintf('MAX Pr with MAX Recall: Pr %f, Re %f, File: %s\n', max_pr, max_re, max_file);

    lab_size = 26;
    ax_size = 20;
    leg_size = 20;
    line_size = 3.0;
    figure();
    hold on;
    set(gca, 'FontSize', ax_size);
    xlabel('Recall', 'FontSize', lab_size);
    ylabel('Precision', 'FontSize', lab_size);
    xlim([0., 1.02]);
    ylim([0., 1.02]);
    grid('off');
    plot(Rs, Ps, 'r');
    hold off
end