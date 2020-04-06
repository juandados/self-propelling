function hwPlotTest(obj)
% function highwayPosTest(obj, z)
% Test routine for hwPlot; plots the highway
%
% Mo Chen, 2015-06-14

disp('Testing hwPlot()')

% Call the function to be tested
figure;
obj.hwPlot
title('Default colored highway')

figure;
obj.hwPlot('b')
title('Blue highway')
end