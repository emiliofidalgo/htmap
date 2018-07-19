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