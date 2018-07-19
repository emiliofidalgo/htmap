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