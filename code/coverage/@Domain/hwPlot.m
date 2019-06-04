function hwPlot(obj, color)
% function hwPlot(obj)
%
% Plots the highway
%
% Inputs: color - color to plot the highway in
% 
% Mo Chen, 2015-06-21

if nargin<2
  color = 'k'; 
end

% call the plot method from the superclass lpPath (linear path)
obj.lpPlot(color);

end