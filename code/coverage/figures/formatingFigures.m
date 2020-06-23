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
l = 50;
m = 50;
xlim([-m l]);
ylim([-m l]);
%xlim([-l l]);
%ylim([-l l]);
st = 15;
set(ax,'xtick',-m:st:l);
set(ax,'ytick',-m:st:l);
time = 48;
title(['t=',num2str(time),' (s)'],'fontSize',18,'Interpreter','latex');
axis square;
box on;
%axis equal;
grid on;
set(gcf,'renderer','Painters')
% plotting path
a = 2*pi/40;
r = 30;
domainPath = @(t) [r*cos(a.*t); r*sin(a.*t)];
%domainPath = @(t) (1/sqrt(2))*[1;1].*t;
path = domainPath([0:0.5:80]);
plot(path(1,:),path(2,:),'--','lineWidth',2,'color','b');
%%
for i = 1:16
    ax.Children(end-2*i).AutoScaleFactor=0;
end
