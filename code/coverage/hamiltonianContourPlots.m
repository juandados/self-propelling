% the hamiltonian is max_u min_d gradV.f =
% max_u min_d{
%  gradV(1)*vxr + ux*gradV(2) - dx*gradV(2)
% +gradV(3)*vyr + uy*gradV(4) - dy*gradV(4)
% }
close all;
for i = 1:100
    base_x = 2*rand(4, 1);
    %base_x = [-5;0;0;1];
    grad_t = {grad{1}(:,:,:,:,end); grad{2}(:,:,:,:,end); ...
        grad{3}(:,:,:,:,end); grad{4}(:,:,:,:,end)};
    base_grad = eval_u(g, grad_t, base_x);
    u_max = 3;
    ux = [-u_max:0.1:u_max];
    uy = [-u_max:0.1:u_max];
    [Ux, Uy] = meshgrid(ux, uy);
    dx = -base_grad(2)/(norm(base_grad([2,4]))+eps); dy = -base_grad(4)/(norm(base_grad([2,4]))+eps);
    H = base_grad(1)*base_x(2) + Ux*base_grad(2) - dx*base_grad(2)...
        + base_grad(3)*base_x(4) + Uy*base_grad(4) - dy*base_grad(4);
    dt = 0.05;
    dV_dt = -(data(:,:,:,:,end) - data(:,:,:,:,end-1))/dt; % check the order because time is going backwards
    base_dV_dt = eval_u(g, dV_dt, base_x);
    change = H + base_dV_dt;
    disp(i)
    if max(change(:)) < 0
        figure; hold on;
        contourf(Ux, Uy, change); colorbar;
        t = [0:0.01:2*pi];
        r = u_max*[cos(t); sin(t)];
        vel_grad = [base_grad(2), base_grad(4)];
        op = u_max * vel_grad / norm(vel_grad) ;
        plot(r(1,:), r(2,:),'r');
        plot(op(1), op(2), 'r*');
        keyboard
    end
    %

end