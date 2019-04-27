clear all;
pw = 0.1;
l = 16;
fh = @(h)(1*h.^2+l*h/2).*(h>0)+200*(h.*sin(h*pi/pw)).*(h<pw).*(h>0);
h = [-1:0.001:5];
f = fh(h);
plot(h,f);