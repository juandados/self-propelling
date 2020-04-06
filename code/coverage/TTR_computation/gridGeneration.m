function [xs, N] = gridGeneration(dim, min, max, dx)
    x1gv = min(1):dx(1):max(1);
    x2gv = min(2):dx(2):max(2);
    x3gv = min(3):dx(3):max(3);
    x4gv = min(4):dx(4):max(4);
    [xs(:,:,:,:,1), xs(:,:,:,:,2), xs(:,:,:,:,3), xs(:,:,:,:,4)] = ndgrid(x1gv, x2gv, x3gv, x4gv);
    N = size(xs);
    N = N(1:end-1);
end