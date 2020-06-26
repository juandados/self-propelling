%% 1.1 Static domain (avoidance off)
extraArgs.N = 16;
extraArgs.avoidance = false;
extraArgs.domainType = 'squarePaper';
extraArgs.vd = @(t)[0;0]; %domain velocity
extraArgs.domainPath = @(t) t*extraArgs.vd(t);
extraArgs.domainRotationAngle = @(t) 0;
extraArgs.initialConfig = 'line';
extraArgs.c_al = 0; %(inter-vehicle velocity alignment stregth)
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 2; %(same as beta: non zero slope in fh)
extraArgs.a_v = 0.2; %(vehicle-domain velocity alignment strength )
extraArgs.tMax = 60;
simulateMultiVehicle(true, true, extraArgs)
%% 1.2 Static domain plot (avoidance on)
extraArgs.N = 16;
extraArgs.avoidance = true;
extraArgs.domainType = 'squarePaper';
extraArgs.vd = @(t)[0;0]; %domain velocity
extraArgs.domainPath = @(t) t*extraArgs.vd(t);
extraArgs.domainRotationAngle = @(t) 0;
extraArgs.initialConfig = 'line';
extraArgs.c_al = 0; %(inter-vehicle velocity alignment stregth)
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 2; %(same as beta: non zero slope in fh)
extraArgs.a_v = 0.2; %(vehicle-domain velocity alignment strength )
extraArgs.tMax = 60;
simulateMultiVehicle(true, true, extraArgs)
%% 2. Static domain table (N: 9-16-25)
% basic setup
extraArgs.domainType = 'squarePaper';
extraArgs.vd = @(t)[0;0]; %domain velocity
extraArgs.domainPath = @(t) t*extraArgs.vd(t);
extraArgs.domainRotationAngle = @(t) 0;
extraArgs.initialConfig = 'line';
extraArgs.c_al = 0; %(inter-vehicle velocity alignment stregth)
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 2; %(same as beta: non zero slope in fh)
extraArgs.a_v = 0.2; %(vehicle-domain velocity alignment strength )
extraArgs.tMax = 60;
% variations
extraArgs.N = 9;
extraArgs.avoidance = true;
simulateMultiVehicle(true, true, extraArgs)
extraArgs.avoidance = false;
simulateMultiVehicle(true, true, extraArgs)
extraArgs.N = 16;
extraArgs.avoidance = true;
simulateMultiVehicle(true, true, extraArgs)
extraArgs.avoidance = false;
simulateMultiVehicle(true, true, extraArgs)
extraArgs.N = 25;
extraArgs.avoidance = true;
simulateMultiVehicle(true, true, extraArgs)
extraArgs.avoidance = false;
simulateMultiVehicle(true, true, extraArgs)
%% 3.1 Moving Domain (vehi-aligment on, domain-alignment on, collision off)
extraArgs.N = 10;
extraArgs.avoidance = false;
extraArgs.domainType = 'trianglePaper';
extraArgs.vd = @(t)(1/sqrt(2))*[1;1]; %domain velocity
extraArgs.domainPath = @(t) t*extraArgs.vd(t);
extraArgs.domainRotationAngle = @(t) -pi/4;
extraArgs.initialConfig = 'arrowPaper';
extraArgs.c_al = 0.2; %(inter-vehicle velocity alignment stregth) %1
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 2; %(same as beta: non zero slope in fh)
extraArgs.a_v = 0.2; %(vehicle-domain velocity alignment strength ) %0.2
extraArgs.tMax = 60;
extraArgs.tailSize = 150;
simulateMultiVehicle(true, true, extraArgs)
%% 3.2 Moving Domain (vehi-aligment on, domain-alignment on, collision on)
extraArgs.N = 10;
extraArgs.avoidance = true;
extraArgs.domainType = 'trianglePaper';
extraArgs.vd = @(t)(1/sqrt(2))*[1;1]; %domain velocity
extraArgs.domainPath = @(t) t*extraArgs.vd(t);
extraArgs.domainRotationAngle = @(t) -pi/4;
extraArgs.initialConfig = 'arrowPaper';
extraArgs.c_al = 0.2; %(inter-vehicle velocity alignment stregth) %1
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 2; %(same as beta: non zero slope in fh)
extraArgs.a_v = 0.2; %(vehicle-domain velocity alignment strength ) %0.2
extraArgs.tMax = 60;
extraArgs.tailSize = 150;
simulateMultiVehicle(true, true, extraArgs)
%% 3.3 Moving Domain (vehi-aligment on, domain-alignment on, collision off)
% large a_v
extraArgs.N = 10;
extraArgs.avoidance = false;
extraArgs.domainType = 'trianglePaper';
extraArgs.vd = @(t)(1/sqrt(2))*[1;1]; %domain velocity
extraArgs.domainPath = @(t) t*extraArgs.vd(t);
extraArgs.domainRotationAngle = @(t) -pi/4;
extraArgs.initialConfig = 'arrowPaper';
extraArgs.c_al = 0.2; %(inter-vehicle velocity alignment stregth)
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 2; %(same as beta: non zero slope in fh)
extraArgs.a_v = 1.4; %(vehicle-domain velocity alignment strength ) 1.6
extraArgs.tMax = 60;
extraArgs.tailSize = 150;
simulateMultiVehicle(true, true, extraArgs)
%% 3.4 Moving Domain (vehi-aligment on, domain-alignment on, collision on)
% large C_al
extraArgs.N = 10;
extraArgs.avoidance = false;
extraArgs.domainType = 'trianglePaper';
extraArgs.vd = @(t)(1/sqrt(2))*[1;1]; %domain velocity
extraArgs.domainPath = @(t) t*extraArgs.vd(t);
extraArgs.domainRotationAngle = @(t) -pi/4;
extraArgs.initialConfig = 'arrowPaper';
extraArgs.c_al = 1.4; %(inter-vehicle velocity alignment stregth)
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 2; %(same as beta: non zero slope in fh)
extraArgs.a_v = 0.2; %(vehicle-domain velocity alignment strength )
extraArgs.tMax = 60;
extraArgs.tailSize = 150;
simulateMultiVehicle(true, true, extraArgs)
%% 3.5 Moving Domain (vehi-aligment on, domain-alignment on, collision on)
% samll a_v and C_al it is not being included in the thesis, but the
% effects are commented.
extraArgs.N = 10;
extraArgs.avoidance = true;
extraArgs.domainType = 'trianglePaper';
extraArgs.vd = @(t)(1/sqrt(2))*[1;1]; %domain velocity
extraArgs.domainPath = @(t) t*extraArgs.vd(t);
extraArgs.domainRotationAngle = @(t) -pi/4;
extraArgs.initialConfig = 'arrowPaper';
extraArgs.c_al = 0.035; %(inter-vehicle velocity alignment stregth)
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 2; %(same as beta: non zero slope in fh)
extraArgs.a_v = 0.035; %(vehicle-domain velocity alignment strength )
extraArgs.tMax = 60;
extraArgs.tailSize = 150;
simulateMultiVehicle(true, true, extraArgs)
%% 4.1 Moving arrow (comparing old with new: must reduce oscilations)
extraArgs.N = 9;
extraArgs.avoidance = true;
extraArgs.domainType = 'arrowPaper';
tl = 80;
extraArgs.vd = @(t)1*((1/sqrt(2))*[1;1]*(t<=tl) + [1;0]*(t>tl)); %domain velocity
extraArgs.domainPath = @(t) t*extraArgs.vd(t)*(t<=tl) + (extraArgs.vd(tl)*tl+...
    +(t-tl)*extraArgs.vd(t))*(t>tl);
extraArgs.domainRotationAngle = @(t) -pi/4*(t>tl);
extraArgs.initialConfig = 'arrowPaper';
extraArgs.c_al = 0.1; %(inter-vehicle velocity alignment stregth)%0.5
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 2; %(same as beta: non zero slope in fh)
extraArgs.a_v = 0.2; %(vehicle-domain velocity alignment strength )
extraArgs.tMax = 60;
extraArgs.tailSize = 200;
tm = simulateMultiVehicle(true, true, extraArgs);
%% x.1 Moving Domain non inertial Parabola
extraArgs.N = 6;
extraArgs.avoidance = true;
extraArgs.domainType = 'trianglePaper';
a = 0.008*2;
extraArgs.vd = @(t)[1/sqrt(1+(2*a*t)^2); 2*a*t/sqrt(1+(2*a*t)^2)];
extraArgs.domainPath = @(t) [asinh(2*a*t)/(2*a);sqrt((2*a*t)^2+1)/(2*a)];
extraArgs.domainRotationAngle = @(t)atan2(2*a*t./sqrt(1+(2*a*t)^2),1/sqrt(1+(2*a*t)^2))-pi/2;
extraArgs.initialConfig = 'arrowPaper';
extraArgs.c_al = 1; %(inter-vehicle velocity alignment stregth)
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 2; %(same as beta: non zero slope in fh)
extraArgs.a_v = 0.2; %(vehicle-domain velocity alignment strength )
extraArgs.tMax = 70;
simulateMultiVehicle(true, true, extraArgs)
%% 4.2 Moving Domain non inertial Circle case
extraArgs.N = 6;
extraArgs.avoidance = true; %true
extraArgs.domainType = 'trianglePaper';
a = 2*pi/40;
r = 30;
extraArgs.vd = @(t)[-a*r*sin(a*t); a*r*cos(a*t)];
extraArgs.domainPath = @(t) [r*cos(a*t); r*sin(a*t)];
extraArgs.domainRotationAngle = @(t)atan2(a*r*cos(a*t),-a*r*sin(a*t))-pi/2;
extraArgs.initialConfig = 'arrowPaper';
extraArgs.c_al = 1.2; %(inter-vehicle velocity alignment stregth) %1
extraArgs.a_I = 20; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 10; %(same as beta: non zero slope in fh) % 5
extraArgs.a_v = 1; %(vehicle-domain velocity alignment strength ) %1
extraArgs.tMax = 80;
extraArgs.tailSize = 200;
tm = simulateMultiVehicle(true, true, extraArgs);
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%~~~~~~~~~~~~~~~ Dubins Car ~~~~~~~~~~~~~~~~~~~
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3.2 Moving Domain (vehi-aligment on, domain-alignment on, collision on)
extraArgs.N = 10;
extraArgs.avoidance = true;
extraArgs.domainType = 'trianglePaper';
extraArgs.vd = @(t)(1/sqrt(2))*[1;1]; %domain velocity
extraArgs.domainPath = @(t) t*extraArgs.vd(t);
extraArgs.domainRotationAngle = @(t) -pi/4;
extraArgs.initialConfig = 'arrowPaper';
extraArgs.c_al = 0.2; %(inter-vehicle velocity alignment stregth) %1
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 2; %(same as beta: non zero slope in fh)
extraArgs.a_v = 0.2; %(vehicle-domain velocity alignment strength ) %0.2
extraArgs.tMax = 60;
extraArgs.tailSize = 150;
simulateMultiDubins(true, true, extraArgs)
%% 4.1 Moving arrow (comparing old with new: must reduce oscilations)
extraArgs.N = 9;
extraArgs.avoidance = true;
extraArgs.domainType = 'arrowPaper';
tl = 80;
extraArgs.vd = @(t)1*((1/sqrt(2))*[1;1]*(t<=tl) + [1;0]*(t>tl)); %domain velocity
extraArgs.domainPath = @(t) t*extraArgs.vd(t)*(t<=tl) + (extraArgs.vd(tl)*tl+...
    +(t-tl)*extraArgs.vd(t))*(t>tl);
extraArgs.domainRotationAngle = @(t) -pi/4*(t>tl);
extraArgs.initialConfig = 'arrowPaper';
extraArgs.c_al = 0.4; %(inter-vehicle velocity alignment stregth)%0.5
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 2; %(same as beta: non zero slope in fh)
extraArgs.a_v = 0.4; %(vehicle-domain velocity alignment strength )
extraArgs.tMax = 60;
extraArgs.tailSize = 200;
tm = simulateMultiDubins(true, true, extraArgs);
%% 4.2 Moving Domain non inertial Circle case
extraArgs.N = 6;
extraArgs.avoidance = false; %true
extraArgs.domainType = 'trianglePaper';
a = 2*pi/40;
r = 30;
extraArgs.vd = @(t)[-a*r*sin(a*t); a*r*cos(a*t)];
extraArgs.domainPath = @(t) [r*cos(a*t); r*sin(a*t)];
extraArgs.domainRotationAngle = @(t)atan2(a*r*cos(a*t),-a*r*sin(a*t))-pi/2;
extraArgs.initialConfig = 'arrowPaper';
extraArgs.c_al = 0.2; %(inter-vehicle velocity alignment stregth) %1
extraArgs.l_al_decay = 0.5;
extraArgs.a_I = 5; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 3.2; %(same as beta: non zero slope in fh) % 5
extraArgs.a_v = 1.5; %(vehicle-domain velocity alignment strength ) %1
extraArgs.tMax = 80;
extraArgs.tailSize = 200;
tm = simulateMultiDubins(true, true, extraArgs);