function [sThis, sOther] = hwInt(obj, other)
% function hwInt(obj, other)
%
% Computes the intersection of two highways
%
% Outputs: sThis  - highway parameter for current highway "obj"
%          sOther - highway parameter for the other highway "other"
%             both are empty if highways don't intersect
% 
% Mo Chen, 2015-05-25

% Unpack highway endpoints for convenience
x1 = obj.z0(1); 
y1 = obj.z0(2);

x2 = obj.z1(1); 
y2 = obj.z1(2);

x3 = other.z0(1); 
y3 = other.z0(2);

x4 = other.z1(1); 
y4 = other.z1(2);

% MATLAB's built-in function for line intersection (returns empty if
% intersection is not found
xi= polyxpoly([x1 x2], [y1 y2], [x3 x4], [y3 y4]);

% Compute highway parameters (automatically empty if intersection not
% found)
sThis = (xi-x1) / (x2-x1);
sOther = (xi-x3) / (x4-x3);

end