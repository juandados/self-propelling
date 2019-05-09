clear all;
pw = 0.1;
l = 16;
fh = @(h)(1*h.^2+l*h/2).*(h>0)+200*(h.*sin(h*pi/pw)).*(h<pw).*(h>0);
h = [-1:0.001:5];
f = fh(h);
plot(h,f);
%%
thr = 10;
rd = 0.1;
fI = @(r)max(min(10*(r-rd).*(r-rd<0),thr),-thr);
pw = rd;
l = 10;
fh = @(h)max(min((1*h.^2+l*h/2).*(h>0)+80*(h.*sin(h*pi/pw)).*(h<pw).*(h>0),thr),-thr);
% without Threshold
rd = 0.1;
fIn = @(r)10*(r-rd).*(r-rd<0);
pw = rd;
l = 10;
fhn = @(h)(1*h.^2+l*h/2).*(h>0)+80*(h.*sin(h*pi/pw)).*(h<pw).*(h>0);
x = [-10:0.1:10];
close all; figure();
subplot(1,2,1); hold on; plot(x,fI(x)); plot(x,fIn(x),'--'); xlabel('r'); ylabel('fI');
subplot(1,2,2); hold on; plot(x,fh(x)); plot(x,fhn(x),'--'); xlabel('h'); ylabel('fh');