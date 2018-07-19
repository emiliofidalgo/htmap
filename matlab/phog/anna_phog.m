function p = anna_phog(I,bin,angle,L,roi)
% anna_PHOG Computes Pyramid Histogram of Oriented Gradient over a ROI.
%               
% [BH, BV] = anna_PHOG(I,BIN,ANGLE,L,ROI) computes phog descriptor over a ROI.
% 
% Given and image I, phog computes the Pyramid Histogram of Oriented Gradients
% over L pyramid levels and over a Region Of Interest

%IN:
%	I - Images of size MxN (Color or Gray)
%	bin - Number of bins on the histogram 
%	angle - 180 or 360
%   L - number of pyramid levels
%   roi - Region Of Interest (ytop,ybottom,xleft,xright)
%
%OUT:
%	p - pyramid histogram of oriented gradients

Img = imread(I);
if size(Img,3) == 3
    G = rgb2gray(Img);
else
    G = Img;
end
bh = [];
bv = [];

if sum(sum(G))>100
    E = edge(G,'canny');
    [GradientX,GradientY] = gradient(double(G));
    GradientYY = gradient(GradientY);
    Gr = sqrt((GradientX.*GradientX)+(GradientY.*GradientY));
            
    index = GradientX == 0;
    GradientX(index) = 1e-5;
            
    YX = GradientY./GradientX;
    if angle == 180, A = ((atan(YX)+(pi/2))*180)/pi; end
    if angle == 360, A = ((atan2(GradientY,GradientX)+pi)*180)/pi; end
                                
    [bh bv] = anna_binMatrix(A,E,Gr,angle,bin);
    
    %%%%% Show images  
    close all;
    [h, w] =  size(E)
    
    % Original image
    figure, imshow(G);
    %title('Original Image', 'FontSize', 14);
    
    print -dpng -r300 phog_orig
    
    % L = 0
    figure, imshow(E);
    set(gca,'position',[0 0 1 1],'units','normalized')
    %title('L = 0', 'FontSize', 14);
    print -dpng -r300 phog_l0
    
    % L = 1
    figure; imshow(E);   
    %title('L = 1', 'FontSize', 14);
    hold on;        
    plot([w/2, w/2], [1, h], 'g', 'LineWidth',5);
    plot([1, w], [h/2, h/2], 'g', 'LineWidth',5);
    set(gca,'position',[0 0 1 1],'units','normalized')
    hold off;
    print -dpng -r300 phog_l1    
    
    % L = 2
    figure; imshow(E);
    %title('L = 2', 'FontSize', 14);
    hold on;        
    plot([w/2, w/2], [1, h], 'g', 'LineWidth',5);            
    plot([1, w], [h/2, h/2], 'g', 'LineWidth',5);
    
    plot([w/4 + w/2, w/4 + w/2], [1, h], 'g', 'LineWidth',5);
    plot([w/4, w/4], [1, h], 'g', 'LineWidth',5);
    
    plot([1, w], [h/4, h/4], 'g', 'LineWidth',5);
    plot([1, w], [h/4 + h/2, h/4 + h/2], 'g', 'LineWidth',5);
    
    set(gca,'position',[0 0 1 1],'units','normalized')
    hold off;
    print -dpng -r300 phog_l2
    %%%%% End Show images
else
    bh = zeros(size(I,1),size(I,2));
    bv = zeros(size(I,1),size(I,2));
end

bh_roi = bh(roi(1,1):roi(2,1),roi(3,1):roi(4,1));
bv_roi = bv(roi(1,1):roi(2,1),roi(3,1):roi(4,1));
p = anna_phogDescriptor(bh_roi,bv_roi,L,bin);

s = sprintf('%s.txt',I);
dlmwrite(s,p);
