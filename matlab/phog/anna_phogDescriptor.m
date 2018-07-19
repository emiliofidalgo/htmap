function p = anna_PhogDescriptor(bh,bv,L,bin)
% anna_PHOGDESCRIPTOR Computes Pyramid Histogram of Oriented Gradient over a ROI.
%               
%IN:
%	bh - matrix of bin histogram values
%	bv - matrix of gradient values 
%   L - number of pyramid levels
%   bin - number of bins
%
%OUT:
%	p - pyramid histogram of oriented gradients (phog descriptor)

p = [];
%level 0
for b=1:bin
    ind = bh==b;
    p = [p;sum(bv(ind))];
end
        
cella = 1;
for l=1:L
    x = fix(size(bh,2)/(2^l));
    y = fix(size(bh,1)/(2^l));
    xx=0;
    yy=0;
    while xx+x<=size(bh,2)
        while yy +y <=size(bh,1) 
            bh_cella = [];
            bv_cella = [];
            
            bh_cella = bh(yy+1:yy+y,xx+1:xx+x);
            bv_cella = bv(yy+1:yy+y,xx+1:xx+x);
            
            for b=1:bin
                ind = bh_cella==b;
                p = [p;sum(bv_cella(ind))];
            end 
            yy = yy+y;
        end        
        cella = cella+1;
        yy = 0;
        xx = xx+x;
    end
end
if sum(p)~=0
    p = p/sum(p);
end

