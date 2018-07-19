close all

load('fGPS.mat');
load('groundtruth.mat');

plot(fGPS(:,1), fGPS(:,2));
hold on;

[rows, cols] = find(truth);
for i=1:length(rows)
    row = rows(i);
    col = cols(i);
    plot([fGPS(row, 1), fGPS(col, 1)], [fGPS(row, 2), fGPS(col, 2)], 'r*-');
    
    disp([num2str((i/length(rows) * 100)), ' completed']);
    
    %pause(0.005);
end

hold off