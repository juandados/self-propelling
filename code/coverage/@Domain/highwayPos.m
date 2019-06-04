function [s, dist, x] = highwayPos(obj, z)
% function [s, dist, x] = highwayPos(obj, z)
%
% Returns position on highway given absolute position z
% (orthogonal projection)
%
% WARNING: THERE'S SOMETHING WRONG... doesn't work in 
%          simulateHighwayMerge.m 
%
% Inputs:  obj  - highway object
%          z    - absolute position (eg. current vehicle position)
%
% Outputs: s    - location on highway (highway parameter between 0 and 1)
%          dist - distance from z nearest location on highway
%          x    - absolute location on highway
% 
% Mo Chen, 2015-06-15

% Unpack endpoints for convenience
x1 = obj.z0(1); y1 = obj.z0(2);
x2 = obj.z1(1); y2 = obj.z1(2);

% Perpendicular directions
dx_perp = -obj.ds(2); 
dy_perp = obj.ds(1);

% "Origin" of input point
x3 = z(1);
y3 = z(2);

% Extend input point in both directions along perpendicular direction
large = 1e3;
x3 = x3 - 0.5*large*dx_perp;
y3 = y3 - 0.5*large*dy_perp;

x4 = x3 + large*dx_perp;
y4 = y3 + large*dy_perp;

% Compute intersection using built-in MATLAB function and then compute s
%[xi, yi] = polyxpoly([x1 x2], [y1 y2], [x3 x4], [y3 y4]); %Juan comment
xi = 0; % Juan Mod
yi = 0; % Juan Mod
if ~isempty(xi)
    s = (xi-x1) / (x2-x1);
    dist = sqrt((xi-z(1))^2 + (yi-z(2))^2);
else
    dist1 = sqrt((x1-z(1))^2 + (y1-z(2))^2);
    dist2 = sqrt((x2-z(1))^2 + (y2-z(2))^2);
    
    if dist1 < dist2
        dist = dist1;
        s = 0;
    else
        dist = dist2;
        s = 1;
    end
end

x = [xi yi];
% return
% % Compute this numerically so that it's general enough for
% % curves as well
% 
% tol = 0.05;           % Distance tolerance
% N = 10;               % Discretization
% ss = linspace(0,1,N); % Discretized s parameters
% ff = obj.fn(ss);      % Discretized highway locations
% 
% % Discretization size
% df = [ff(1,2) - ff(1,1); ff(2,2) - ff(2,1)];
% 
% % Iterate until tolerance is met
% while norm(df) > tol
%     %                 disp('...')
%     N = 2*N;
%     ss = linspace(0,1,N); % Discretized s parameters
%     ff = obj.fn(ss);      % Discretized highway locations
%     
%     % Discretization size
%     df = [ff(1,2) - ff(1,1); ff(2,2) - ff(2,1)];
% end
% 
% % Distance from z to every discretized location
% dz1 = z(1) - ff(1,:);
% dz2 = z(2) - ff(2,:);
% dist = sqrt(dz1.^2 + dz2.^2);
% 
% % Minimum distance
% [dist, si] = min(dist);
% s = ss(si);


end


