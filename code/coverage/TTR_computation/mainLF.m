function phi = mainLF(min, max, dx)
    global UMax;
    global DMax;

    UMax = 3; DMax = 3;

    epsilon = 1e-6;

    % z = [px vx py vy]
    dim = 4;
    r=1;
    if nargin < 1
        min = r*[-10, -5, -10, -5];
    end
    if nargin < 2
        max = r*[10, 5, 10, 5];
    end
    if nargin < 3
        dx = 0.8*r*[20/49, 10/49, 20/49, 10/49];
    end
    
    [xs, N] = gridGeneration(dim, min, max, dx);

    d = 2.5; %collision radius it must be a parameter
    phi = 100 * (xs(:,:,:,:,1).^2 + xs(:,:,:,:,3).^2 >= d^2);
    visSetIm({},phi);

    %LF sweeping
    mex 'TTR_computation/mexLFsweep.cpp';
    disp('mexing done!');

    numIter = 50;
    TOL = 1e-6;

    startTime = cputime;
    tic;
    mexLFsweep(phi, xs, dx, UMax, DMax, numIter, TOL);
    toc;

    endTime = cputime;
    fprintf('Total execution time %g seconds', endTime - startTime);
end