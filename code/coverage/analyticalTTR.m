%% Analitical solution
clear all;
cr = 2.5;
a = @(vx,vy) vx.^2+vy.^2;
b = @(px,vx,py,vy) 2*px.*vx+2*py.*vy;
c = @(px,py) px.^2+py.^2-cr.^2;
t1 = @(px,vx,py,vy) (-b(px,vx,py,vy)-...
    sqrt(b(px,vx,py,vy).^2-4*a(vx,vy).*c(px,py))) ...
    ./(2*a(vx,vy));
t2 = @(px,vx,py,vy) (-b(px,vx,py,vy)+...
    sqrt(b(px,vx,py,vy).^2-4*a(vx,vy).*c(px,py))) ...
    ./(2*a(vx,vy));
phi = @(px,vx,py,vy) t1(px,vx,py,vy).*(t1(px,vx,py,vy)>=0) ...
    + 100*(t2(px,vx,py,vy)<=0);
%% Evaluating in grid
g.min = [-10, -2, -10, -2];
g.max = [10, 2, 10, 2];
g.N = floor(2*[50, 50, 50, 50]);
g.dx = (g.max-g.min)./(g.N-1);
g = createGrid(g.min,g.max, g.N);
ttr = phi(g.xs{1},g.xs{2},g.xs{3},g.xs{4});
ttr((imag(ttr)~=0))=100;
%% Visualizing
close all;
figure();
visSetIm(g,ttr,'red',0.5);
figure();
[gxy, dataxy] = proj(g, ttr, [0 1 0 1], [0 1]);
visSetIm(gxy, dataxy, 'red', 0:0.1:2);