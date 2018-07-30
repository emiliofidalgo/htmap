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

function distances = computeGPSDist(lat, long)
    [r, c] = size(long);
    distances = zeros(r, r);
    for i=1:r  
        disp(['Processing image ', num2str(i)]);
        parfor j=1:i
            lon1 = long(i,1);
            lat1 = lat(i,1);
            lon2 = long(j,1);
            lat2 = lat(j,1);            
            [km, nmi, mi] = haversine([lat1 lon1], [lat2 lon2]);            
            dist_m = km * 1000.0;
            distances(i,j) = dist_m;            
        end
    end     
end