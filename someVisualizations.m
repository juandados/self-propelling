%% visualizing implicit sets
close all;
figure();
visSetIm(g, data, 'red', 1);
figure();
[g3D, data3D] = proj(g, data, [0 0 0 1], [5]);
visSetIm(g3D, data3D, 'red', 1);
%% Visualizing a 2D slice: It is removing the velocities by slicing the vx =
% 0.5, vy = 0.5
close all;
figure();
visSetIm(g, data, 'red', 1);
figure();
[g3D, data3D] = proj(g, data, [1 0 1 0], [-6, 4]);
contourf(g3D.xs{1}, g3D.xs{2}, data3D);
colorbar
%%
surf(g3D.xs{1}, g3D.xs{2}, data3D);
shading interp;
figure();
contourf(g3D.xs{1}, g3D.xs{2}, data3D);
colorbar;
% see the level sets
%close all;
figure()
for i = 0:1:100
    hold on;
    visSetIm(g3D, data3D, 'red', i);
end
