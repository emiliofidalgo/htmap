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

function computeTotalDistanceGPS(infile)
    poses = load(infile);    
    [r, c] = size(poses.fGPS);
    total_distance = 0.0;
    for i=2:r
        lat1 = poses.fGPS(i - 1, 1);
        lon1 = poses.fGPS(i - 1, 2);
        lat2 = poses.fGPS(i, 1);
        lon2 = poses.fGPS(i, 2);
        [km, nmi, mi] = haversine([lat1 lon1], [lat2 lon2]);
        dist_m = km * 1000.0;
        total_distance = total_distance + dist_m;
    end    
    disp(num2str(total_distance));
end