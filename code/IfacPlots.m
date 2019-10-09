close all; clear all;
fontSize = 40;
rd = 3;
fI = @(r)1*(r-rd).*(r-rd<0);
xmin = -1; xmax=7; ymin=-rd; ymax=0.5*rd;
r = [0:xmax];
figure(1);
plot(r,fI(r),'k','lineWidth',4);
axis([xmin xmax ymin ymax]);
axh = gca; % use current axes
% axis lines
line(get(axh,'XLim'), [0 0], 'Color', 'k','lineWidth',2);
line([0 0], get(axh,'YLim'), 'Color', 'k','lineWidth',2);
% labels
text(xmax*0.75, 0.4,'$\left\Vert r_{ij}\right\Vert$','fontSize',fontSize, 'Interpreter','latex');
text(rd, 0.35,'$r_{d}$','fontSize',fontSize, 'Interpreter','latex');
text(-1.1, ymax*0.8,'$f_{I}$','fontSize',fontSize, 'Interpreter','latex');
set(gca, 'visible', 'off');
% thight axis
outerpos = axh.OuterPosition;
ti = axh.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
axh.Position = [left bottom ax_width ax_height];
%%
close all; clear all;
fontSize = 40;
rd = 3;
fh = @(h)1*(h+rd/2).*((h+rd/2)>0);
xmin = -5; xmax=3; ymin=-0.3*rd; ymax=1.5*rd;
r = [xmin-1:0.05:xmax];
figure(1);
plot(r,fh(r),'k','lineWidth',4);
axis([xmin xmax ymin ymax]);
axh = gca; % use current axes
%axis lines
line(get(axh,'XLim'), [0 0], 'Color', 'k', 'lineWidth', 2);
line([0 0], get(axh,'YLim'), 'Color', 'k', 'lineWidth', 2);
%lables
text(-1, ymax*0.9,'$f_{h}$','fontSize',fontSize, 'Interpreter','latex');
text(xmax*0.5, 0.45,'$\left\Vert h_{i}\right\Vert$','fontSize',fontSize, 'Interpreter','latex');
text(-rd/2-0.5, -0.45,'$-\frac{r_{d}}{2}$','fontSize',fontSize, 'Interpreter','latex')
set(gca, 'visible', 'off');
% thight axis
outerpos = axh.OuterPosition;
ti = axh.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
axh.Position = [left bottom ax_width ax_height];