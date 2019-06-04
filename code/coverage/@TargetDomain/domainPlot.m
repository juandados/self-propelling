function domainPlot(obj, faceColor, color)
% function hwPlot(obj)
%
% Plots the Domain
%
% Inputs: color - color to plot the domain in
% 
% Juan Chacon, 2015-06-21

if nargin<2
  faceColor = 'green'; 
end

if nargin<3
  color = 'k'; 
end

obj.h = plot(obj.polyshape, 'FaceColor', faceColor, 'FaceAlpha', 0.05);
obj.h.EdgeColor = color;

end