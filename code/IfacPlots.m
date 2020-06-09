%%
% vertX = [-sqrt(3)/2 0 sqrt(3)/2 -sqrt(3)/2];
% vertY = [-1/2, sqrt(3)/2 -1/2 -1/2];
vertX = [-1.1 0 1.5];
vertY = [-0.4, sqrt(3)/2 -0.7];
pi = [2,1];
pj = [0.05,2];

close all;
%set(gcf, 'Position',  [100, 100, 450, 450]);
figure(1); hold on;
plot(vertX,vertY,'k','LineWidth',3);
[hi, x_proji,y_proji] = poly_dist(pi(1),pi(2),vertX,vertY);
[hj, x_projj,y_projj] = poly_dist(pj(1),pj(2),vertX,vertY);
s = 30;
plot(pi(1),pi(2),'.k','MarkerSize',s);
plot(pj(1),pj(2),'.k','MarkerSize',s);
plot(x_proji,y_proji,'.k','MarkerSize',s);
plot(x_projj,y_projj,'.k','MarkerSize',s);
plot([x_proji,pi(1)],[y_proji,pi(2)],'--k','LineWidth',3);
plot([x_projj,pj(1)],[y_projj,pj(2)],'--k','LineWidth',3);
plot([pi(1),pj(1)],[pi(2),pj(2)],'--k','LineWidth',3);
Hi=([x_proji,y_proji]-pi)/hi;
Hj=([x_projj,y_projj]-pj)/hj;
pij = pi-pj;
ui = Hi+0.2*pij;
uj = Hj-0.2*pij;
k = 0.5;
quiver([pi(1),pj(1)],[pi(2),pj(2)],k*[Hi(1),Hj(1)],k*[Hi(2),Hj(2)],0,'linewidth',3);
quiver([pi(1),pj(1)],[pi(2),pj(2)],2*k*[ui(1),uj(1)],2*k*[ui(2),uj(2)],0,'linewidth',3);
quiver([pi(1),pj(1)],[pi(2),pj(2)],k*[pij(1),-pij(1)]/norm(pij),k*[pij(2),-pij(2)]/norm(pij),0,'linewidth',3);
%axis([-0.2,1.1,0.2,1.6])
%set(gca, 'visible', 'off');
box on;
text(pi(1),0.2+pi(2),'$p_i$', 'Interpreter','latex','fontSize',24);
text(pj(1),0.2+pj(2),'$p_j$', 'Interpreter','latex','fontSize',24);
text(-1+x_projj,y_projj,'$P_{\partial \Omega}(p_j)$', 'Interpreter','latex','fontSize',24);
text(0.15+x_proji,-0.05+y_proji,'$P_{\partial \Omega}(p_i)$', 'Interpreter','latex','fontSize',24);
text(0.05+pj(1),-0.6+pj(2),'-$\frac{h_{j}}{[[h_{j}]]}$', 'Interpreter','latex','fontSize',25);
text(-0.9+pi(1),-0.1+pi(2),'-$\frac{h_{i}}{[[h_{i}]]}$', 'Interpreter','latex','fontSize',25);
text(0.2+pi(1),-0.5+pi(2),'$\frac{p_{ij}}{\left\Vert p_{ij}\right\Vert}$', 'Interpreter','latex','fontSize',28)
text(-1+pj(1),0.2+pj(2),'$\frac{p_{ji}}{\left\Vert p_{ji}\right\Vert}$', 'Interpreter','latex','fontSize',28)
text(-0.2+pi(1),-0.8+pi(2),'$u_i$', 'Interpreter','latex','fontSize',25)
text(-0.7+pj(1),-0.6+pj(2),'$u_j$', 'Interpreter','latex','fontSize',25)
text(-0.1,-0.2,0,'$\Omega$', 'Interpreter','latex','fontSize',35)
axis equal;
%axis tight;
ax = gca; % use current axes
% axis lines
%axis tight
%xlim(get(ax,'XLim')+[-0.2,0.5]);
%ylim(get(ax,'YLim')+[-0.2,0.35]);
grid on;
set(gca,'xticklabel',[]);
set(gca,'yticklabel',[]);
%axis tight;
%%
set(gca, 'visible', 'off');
axis tight;
%%
close all; clear all;
fontSize = 50;
rd = 3;
fI = @(r)1*(r-rd).*(r-rd<0);
xmin = -2; xmax=7; ymin=-rd; ymax=0.5*rd;
r = [0:xmax];   
figure(1);
plot(r,fI(r),'k','lineWidth',4);
axis([xmin xmax ymin ymax]);
axh = gca; % use current axes
% axis lines
line(get(axh,'XLim'), [0 0], 'Color', 'k','lineWidth',2);
line([0 0], get(axh,'YLim'), 'Color', 'k','lineWidth',2);
% labels
text(xmax*0.95, 0.25,'$r$','fontSize',fontSize, 'Interpreter','latex');
text(rd, 0.35,'$r_{d}$','fontSize',fontSize, 'Interpreter','latex');
text(-2.5, ymax*0.8,'$f_{I}\left(r\right)$','fontSize',0.9*fontSize, 'Interpreter','latex');
set(gca, 'visible', 'off');
% thight axis
outerpos = axh.OuterPosition;
ti = axh.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
axh.Position = [left bottom ax_width ax_height];
% the arrows
xO = 0.2;  
yO = 0.1;
xmax = xmax + xO;
ymax = ymax + yO;
%
patch([xmax-xO -yO; xmax-xO +yO; xmax 0.0], ...
    [yO ymax-xO; -yO ymax-xO; 0 ymax], 'k', 'clipping', 'off');
axis tight;
%% fI
fI = @(r)1*(r-rd).*(r-(rd*b)<0);
close all; clear all;
fontSize = 50;
rd = 3;
fI = @(r)1*(r-rd).*(r-rd<0);
xmin = -2; xmax=7; ymin=-rd; ymax=0.5*rd;
r = [0:xmax];   
figure(1);
plot(r,fI(r),'k','lineWidth',4);
axis([xmin xmax ymin ymax]);
axh = gca; % use current axes
% axis lines
line(get(axh,'XLim'), [0 0], 'Color', 'k','lineWidth',2);
line([0 0], get(axh,'YLim'), 'Color', 'k','lineWidth',2);
% labels
text(xmax*0.95, 0.25,'$r$','fontSize',fontSize, 'Interpreter','latex');
text(rd, 0.35,'$r_{d}$','fontSize',fontSize, 'Interpreter','latex');
text(-2, ymax*0.8,'$f_{I}\left(r\right)$','fontSize',0.9*fontSize, 'Interpreter','latex');
set(gca, 'visible', 'off');
% thight axis
outerpos = axh.OuterPosition;
ti = axh.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
axh.Position = [left bottom ax_width ax_height];
% the arrows
xO = 0.2;  
yO = 0.1;
xmax = xmax + xO;
ymax = ymax + yO;
%
patch([xmax-xO -yO; xmax-xO +yO; xmax 0.0], ...
    [yO ymax-xO; -yO ymax-xO; 0 ymax], 'k', 'clipping', 'off');
axis tight;
%% fh with local min
close all; clear all;
fontSize = 50;
rd = 3;
b = 1;
fh = @(h)1*(h+rd/2).*(h-(-b*rd/2)>=0);
xmin = -5; xmax=3; ymin=-0.3*rd; ymax=1.5*rd;
r = [xmin-1:0.005:xmax];
figure(1);hold on;
plot(r,fh(r),'k.','MarkerSize',15);
plot(r,fh(r),'k:','lineWidth',4);
axis([xmin xmax ymin ymax]);
axh = gca; % use current axes
%axis lines
line(get(axh,'XLim'), [0 0], 'Color', 'k', 'lineWidth', 2);
line([0 0], get(axh,'YLim'), 'Color', 'k', 'lineWidth', 2);
%lables
text(-2.5, ymax*0.9,'$f_{h}\left(r\right)$','fontSize',fontSize*0.9, 'Interpreter','latex');
text(xmax*0.9, 0.35,'$r$','fontSize',fontSize, 'Interpreter','latex');
text(-rd/2-1.6, 0.8,'-$\frac{r_{d}}{2}$','fontSize',fontSize*1.2, 'Interpreter','latex')
%text(-rd/2-2.5, 0.45,'-$h_{0}$','fontSize',fontSize, 'Interpreter','latex')
set(gca, 'visible', 'off');
% thight axis
outerpos = axh.OuterPosition;
ti = axh.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
axh.Position = [left bottom ax_width ax_height];
% the arrows
xO = 0.2;  
yO = 0.1;
xmax = xmax + xO;
ymax = ymax + yO;
%
patch([xmax-xO -yO; xmax-xO +yO; xmax 0.0], ...
    [yO ymax-xO; -yO ymax-xO; 0 ymax], 'k', 'clipping', 'off');
axis tight;
%% fI with local min
close all; clear all;
fontSize = 40;
rd = 3;
b = 1;
fI = @(r) 1*(r-rd).*(r-(rd*b)<=0);
xmin = -2; xmax=7; ymin=-rd; ymax=0.5*rd;
r = [0:0.005:xmax];   
figure(1);hold on;
plot(r,fI(r),'k.','MarkerSize',15);
plot(r,fI(r),'k:','lineWidth',4);
axis([xmin xmax ymin ymax]);
axh = gca; % use current axes
% axis lines
line(get(axh,'XLim'), [0 0], 'Color', 'k','lineWidth',2);
line([0 0], get(axh,'YLim'), 'Color', 'k','lineWidth',2);
% labels
text(xmax*0.95, 0.25,'$r$','fontSize',fontSize, 'Interpreter','latex');
text(rd-1, 0.35,'$r_{d}$','fontSize',fontSize, 'Interpreter','latex');
%text(rd+1.6, 0.35,'$r_{0}$','fontSize',fontSize, 'Interpreter','latex');
text(-2, ymax*0.8,'$f_{I}\left(r\right)$','fontSize',0.9*fontSize, 'Interpreter','latex');
set(gca, 'visible', 'off');
% thight axis
outerpos = axh.OuterPosition;
ti = axh.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
axh.Position = [left bottom ax_width ax_height];
% the arrows
xO = 0.2;  
yO = 0.1;
xmax = xmax + xO;
ymax = ymax + yO;
%
patch([xmax-xO -yO; xmax-xO +yO; xmax 0.0], ...
    [yO ymax-xO; -yO ymax-xO; 0 ymax], 'k', 'clipping', 'off');
axis tight;

%% VI Intervehicle Potential
close all; clear all;
fontSize = 40;
alpha = 0.5;
rd = 3;
b = 1;
VI = @(r) (alpha/2)*(r-rd).^2.*(r-(rd*b)<=0);
xmin = -1.5; xmax=7; ymin=-0.5; ymax=rd;
r = [0:0.005:xmax];   
figure(1);hold on;
plot(r,VI(r),'k.','MarkerSize',15);
%plot(r,fI(r),'k:','lineWidth',4);
axis([xmin xmax ymin ymax]);
axh = gca; % use current axes
% axis lines
line(get(axh,'XLim'), [0 0], 'Color', 'k','lineWidth',2);
line([0 0], get(axh,'YLim'), 'Color', 'k','lineWidth',2);
% labels
text(xmax*0.8, -0.4,'$\left\Vert x\right\Vert$','fontSize',fontSize, 'Interpreter','latex');
text(rd-1, -0.25,'$r_{d}$','fontSize',fontSize, 'Interpreter','latex');
%text(rd+1.6, 0.35,'$r_{0}$','fontSize',fontSize, 'Interpreter','latex');
text(-2, ymax*0.9,'$V_{I}\left(x\right)$','fontSize',0.9*fontSize, 'Interpreter','latex');
set(gca, 'visible', 'off');
% thight axis
outerpos = axh.OuterPosition;
ti = axh.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
axh.Position = [left bottom ax_width ax_height];
% the arrows
xO = 0.2;  
yO = 0.1;
xmax = xmax + xO;
ymax = ymax + yO;
%
patch([xmax-xO -yO; xmax-xO +yO; xmax 0.0], ...
    [yO ymax-xO; -yO ymax-xO; 0 ymax], 'k', 'clipping', 'off');
axis tight;

%% vehicle Domain Potential
close all; clear all;
fontSize = 40;
beta= 0.05;
rd = 3;
b = 1;
Vh = @(h) (beta/2)*((h+rd/2).^2).*(h-(-b*rd/2)>=0);
xmin = -4; xmax=12; ymin=-0.3*rd; ymax=1.5*rd;
r = [xmin-1:0.005:xmax];   
figure(1);hold on;
plot(r,Vh(r),'k.','MarkerSize',15);
%plot(r,fI(r),'k:','lineWidth',4);
axis([xmin xmax ymin ymax]);
axh = gca; % use current axes
% axis lines
line(get(axh,'XLim'), [0 0], 'Color', 'k','lineWidth',2);
line([0 0], get(axh,'YLim'), 'Color', 'k','lineWidth',2);
% labels
text(xmax*0.4, -0.4,'$[[x - P_{\partial \Omega}\left(x\right)]]$','fontSize',fontSize*0.7, 'Interpreter','latex');
%text(rd-1, -0.25,'$r_{d}$','fontSize',fontSize, 'Interpreter','latex');
text(-rd/2-1.6, 0.8,'-$\frac{r_{d}}{2}$','fontSize',fontSize*1.2, 'Interpreter','latex')
%text(rd+1.6, 0.35,'$r_{0}$','fontSize',fontSize, 'Interpreter','latex');
text(-4, ymax*0.9,'$V_{h}\left(x\right)$','fontSize',0.9*fontSize, 'Interpreter','latex');
set(gca, 'visible', 'off');
% thight axis
outerpos = axh.OuterPosition;
ti = axh.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
axh.Position = [left bottom ax_width ax_height];
% the arrows
xO = 0.2;  
yO = 0.1;
xmax = xmax + xO;
ymax = ymax + yO;
%
patch([xmax-xO -yO; xmax-xO +yO; xmax 0.0], ...
    [yO ymax-xO; -yO ymax-xO; 0 ymax], 'k', 'clipping', 'off');
axis tight;

