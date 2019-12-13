%xlim([-30 30]);
%ylim([-30 30]);
set(gcf, 'Position',  [100, 100, 350, 350]);
axis tight;
ax = gca;
ax.FontSize = 14; 
ax.TickLabelInterpreter='latex';
s = 1*max(abs(ax.XLim(1)-ax.XLim(2)), abs(ax.YLim(1)-ax.YLim(2)))/2;
m = [mean(ax.XLim),mean(ax.YLim)];
xlim([m(1)-s,m(1)+s]);
ylim([m(2)-s,m(2)+s]);
xlabel('x position (m)','fontSize',18,'Interpreter','latex');
ylabel('y position (m)','fontSize',18,'Interpreter','latex');
% l = 12;
xlim([-20 40]);
ylim([-20 40]);
% xlim([-l l]);
% ylim([-l l]);
st = 10;
set(ax,'xtick',-40:st:80);
set(ax,'ytick',-40:st:80);
time = 69;
title(['t=',num2str(time),' (s)'],'fontSize',18,'Interpreter','latex');
axis square;
box on;
%axis equal;
grid on;

%%
for i = 1:16
    ax.Children(end-2*i).AutoScaleFactor=0;
end
