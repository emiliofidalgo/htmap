function [bm bv] = anna_BinMatrix(A,E,G,angle,bin)
% anna_BINMATRIX Computes a Matrix (bm) with the same size of the image where
% (i,j) position contains the histogram value for the pixel at position (i,j)
% and another matrix (bv) where the position (i,j) contains the gradient
% value for the pixel at position (i,j)
%                
%IN:
%	A - Matrix containing the angle values
%	E - Edge Image
%   G - Matrix containing the gradient values
%	angle - 180 or 360%   
%   bin - Number of bins on the histogram 
%	angle - 180 or 360
%OUT:
%	bm - matrix with the histogram values
%   bv - matrix with the graident values (only for the pixels belonging to
%   and edge)

[contorns,n] = bwlabel(E);  
X = size(E,2);
Y = size(E,1);
bm = zeros(Y,X);
bv = zeros(Y,X);

nAngle = angle/bin;

for i=1:n
    [posY,posX] = find(contorns==i);    
    for j=1:size(posY,1)
        pos_x = posX(j,1);
        pos_y = posY(j,1);
        
        b = ceil(A(pos_y,pos_x)/nAngle);
        if b==0, bin= 1; end
        if G(pos_y,pos_x)>0
            bm(pos_y,pos_x) = b;
            bv(pos_y,pos_x) = G(pos_y,pos_x);                
        end
    end
end
