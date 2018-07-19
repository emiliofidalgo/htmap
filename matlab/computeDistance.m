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