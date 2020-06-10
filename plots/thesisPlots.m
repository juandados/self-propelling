%% 1.1 Static domain (avoidance on)
extraArgs.N = 16;
extraArgs.avoidance = 'on';
extraArgs.domain = 'square';
extraArgs.vd = [0;0]; %domain velocity
extraArgs.domain_path = @(t) t*vd;
extraArgs.c_al = 0; %(inter-vehicle velocity alignment stregth)
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 1; %(same as beta: non zero slope in fh)
extraArgs.a_v = 1; %(vehicle-domain velocity alignment strength )
simulateMultiVehicle()
%% 1.2 Static domain plot (avoidance off)
extraArgs.N = 16;
extraArgs.avoidance = 'off';
extraArgs.domain = 'square';
extraArgs.vd = [0;0]; %domain velocity
extraArgs.domain_path = @(t) t*vd;
extraArgs.c_al = 0; %(inter-vehicle velocity alignment stregth)
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 1; %(same as beta: non zero slope in fh)
extraArgs.a_v = 1; %(vehicle-domain velocity alignment strength )
simulateMultiVehicle()
%% 2. Static domain table (N: 9-16-25)
%% 3.1 Moving Domain (avoidance off, vehi-aligment off)
extraArgs.N = 15;
extraArgs.avoidance = 'off';
extraArgs.domain = 'triangle45deg';
extraArgs.vd = (0.3/sqrt(2))*[1;1]; %domain velocity
extraArgs.domain_path = @(t) t*vd;
extraArgs.c_al = 0; %(inter-vehicle velocity alignment stregth)
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 1; %(same as beta: non zero slope in fh)
extraArgs.a_v = 1; %(vehicle-domain velocity alignment strength )
simulateMultiVehicle()
%% 3.2 Moving Domain (avoidance off, vehi-aligment on)
extraArgs.N = 15;
extraArgs.avoidance = 'off';
extraArgs.domain = 'triangle45deg';
extraArgs.vd = (0.3/sqrt(2))*[1;1]; %domain velocity
extraArgs.domain_path = @(t) t*vd;
extraArgs.c_al = 1; %(inter-vehicle velocity alignment stregth)
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 1; %(same as beta: non zero slope in fh)
extraArgs.a_v = 1; %(vehicle-domain velocity alignment strength )
simulateMultiVehicle()
%% 4. Static domain table (N: 10-15-21) comparing aligment on, aligment off, avoidance on.
%% 5. Moving non convex Domain non inertial
extraArgs.N = 15;
extraArgs.avoidance = 'off';
extraArgs.domain = 'triangle45deg';
extraArgs.vd = (0.3/sqrt(2))*[1;1]; %domain velocity
extraArgs.domain_path = @(t) t*vd;
extraArgs.c_al = 1; %(inter-vehicle velocity alignment stregth)
extraArgs.a_I = 1; %(same as alpha: non zero slope in fI)
extraArgs.a_h = 1; %(same as beta: non zero slope in fh)
extraArgs.a_v = 1; %(vehicle-domain velocity alignment strength )
simulateMultiVehicle()
%% 6. Moving arrow (comparing old with new: must reduce oscilations)
% turn on anything
%%
domain = 'square';
vd = 0;
simulateMultiVehicle()
%%
