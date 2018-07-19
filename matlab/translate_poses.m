function translate_poses(poses_file)
    f = load(poses_file);
    [r, c] = size(f);
    GPS = zeros(r * 2, 2);
    
    for i=1:r
        GPS(2 * i - 1,1) = f(i,4);
        GPS(2 * i - 1,2) = f(i,8);
        GPS(2 * i,1) = f(i,4);        
        GPS(2 * i,2) = f(i,8);
    end
    
    save('imageCoords.mat', 'GPS');
end