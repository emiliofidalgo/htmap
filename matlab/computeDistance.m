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

function computeDistance(infile)
    poses = load(infile);    
    poses = poses.p;
    [r, c] = size(poses);
    total_distance = 0.0;
    for i=3:2:r
        x = poses(i,1);
        y = poses(i,2);
        distance = sqrt((x - poses(i - 2, 1))^2 + (y - poses(i - 2, 2))^2);
        total_distance = total_distance + distance;        
    end
    
    disp(num2str(total_distance));
end