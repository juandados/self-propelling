agent = UTMPlane4D([0 -2 0 0], pi/10, 3, 0, [1:4], 5, 0.1);
dt=0.05;
T = 20;
close all;
for i=0:dt:T
    %agent.updateState([pi/10,0], dt);
    agent.updateState([0,0.01],dt);
    extraArgs.Color = 'red';
    extraArgs.ArrowLength = 3; %j prev 1
    extraArgs.LineStyle = 'none';
    extraArgs.LineWidth = 0.1; %j
    extraArgs.tailSize = 30; % -1 for showing the whole tail;
    agent.plotPosition(extraArgs);
    axis([-2,2,-2,2]);
    drawnow
    axis square
end