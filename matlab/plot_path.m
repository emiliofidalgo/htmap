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

function plot_path(loops_file, coords_file)
    loops = load(loops_file);
    %loops = loops.truth;
    coords = load(coords_file);
    % For CC and NC
    coords = coords.GPS(1:2:end, :);
        % For KITTIS
        %c = coords.p;
        % For KITTI 06
        %c(:, 1) = coords.p(:, 2);
        %c(:, 2) = -coords.p(:, 1);
        % For KITTIs
        %coords = c;        
        % For St. Lucia
        %coords = coords.fGPS(1:end, :);
    nimages = length(loops);
    figure();
    hold on;
    axis equal
    for i=2:nimages        
        [loops_index] = find(loops(i, :) > 0);
        if numel(loops_index > 0)% && results(i) ~= 1           
            plot([coords(loops_index(1), 1), coords(i, 1)], [coords(loops_index(1), 2), coords(i, 2)], 'g-', 'LineWidth', 1);
            plot([coords(i - 1, 1), coords(i, 1)], [coords(i - 1, 2), coords(i, 2)], 'r-', 'LineWidth', 3);            
            %plot(coords(loops_index(1), 1), coords(loops_index(1), 2), 'r-', 'LineWidth', 2);
            plot([coords(max(loops_index(1) - 1, 1), 1), coords(loops_index(1), 1)], [coords(max(loops_index(1) - 1, 1), 2), coords(loops_index(1), 2)], 'r-', 'LineWidth', 3);
        else
            plot([coords(i - 1, 1), coords(i, 1)], [coords(i - 1, 2), coords(i, 2)], 'k-', 'LineWidth', 3);
        end
        %pause(0.005);
    end    
    hold off;
    axis off;
    
    print -dpng -r300 image
end
