function [safe, uSafeOptimal, uSafeLeft, uSafeRight, safe_val] = checkPWSafety(obj, i, j)
% function checkPWSafety(obj, i, j)
%          (check pairwise safety)
%
% Checks the safety of active agent i with respect to active agent j, and
% gives a safety-preserving input if necessary
%
% Inputs:  obj - tfm object
%          i   - index of agent for whom safety is checked (evader)
%          j   - index of agent against whom safety is checked (pursuer)
% Outputs: safe - boolean specifying whether agent i is safe with respect
%                 to agent j
%          uSafe - safety-preserving input (empty if not needed)
%
% Mo Chen, 2015-11-04
% Modified: Mo Chen, 2015-12-13

% Vehicle is safe with respect to itself
if isempty(obj.aas{j}) || obj.aas{i} == obj.aas{j}
  safe = 1;
  uSafeOptimal = [];
  uSafeLeft = [];
  uSafeRight = [];
  safe_val = 10;
  return
end

switch(class(obj.aas{i}))
  case 'UTMQuad4D'
    %% agent i is a quadrotor
    switch(class(obj.aas{j}))
      case 'UTMQuad4D'
        [safe,  uSafeOptimal, uSafeLeft, uSafeRight, safe_val] = checkPWSafety_qr_qr(obj.qr_qr_safe_V, obj.safetyTime, obj.aas{i}, obj.aas{j});
      otherwise
        error('Unknown agent type')
        
    end % end inner switch
  otherwise
    error('Unknown agent type')
end % end outer switch

end % end function

%%
function [safe, uSafeOptimal, uSafeLeft, uSafeRight, valuex] = checkPWSafety_qr_qr(qr_qr_safe_V, safetyTime, evader, pursuer)
% Safety of quadrotor qr1 with respect to quadrotor qr2

% Heading of evader
theta = evader.getHeading;

% Get relative state assuming pursuer faces 0 degrees
base_pos = rotate2D(evader.getPosition - pursuer.getPosition, -theta);
base_vel = rotate2D(evader.getVelocity - pursuer.getVelocity, -theta);
base_x = [base_pos(1); base_vel(1); base_pos(2); base_vel(2)];

% Check if state is within grid bounds for a closer safety check
if any(base_x <= qr_qr_safe_V.g.min) || ...
    any(base_x >= qr_qr_safe_V.g.max)
  safe = 1;
  uSafeOptimal = [];
  uSafeLeft = [];
  uSafeRight = [];
  valuex = 10;
  return
end

% Compute safety value
% juan:
V_t = qr_qr_safe_V.data(:,:,:,:,1);
valuex = eval_u(qr_qr_safe_V.g, V_t, base_x);

% Compute safety preserving control if needed
if valuex > safetyTime %%|| rand > 0.98
  % Safe; no need to worry about computing controller
  safe = 1;
  uSafeOptimal = [];
  uSafeLeft = [];
  uSafeRight = [];
  valuex = 10;
  return
end

% Not safe, compute safety controller
safe = 0;

% Compute control assuming "pursuer" is facing 0 degrees
% base_grad = calculateCostate(qr_qr_safe_V.g, qr_qr_safe_V.grad, base_x);
% In the help of calculateCostate it says that it is not needed anymore, because it could be replaced by eval_u

%juan:
grad_t = {qr_qr_safe_V.grad{1}(:,:,:,:,1); qr_qr_safe_V.grad{2}(:,:,:,:,1); ...
    qr_qr_safe_V.grad{3}(:,:,:,:,1); qr_qr_safe_V.grad{4}(:,:,:,:,1)};
base_grad = eval_u(qr_qr_safe_V.g, grad_t , base_x);

% Controller1: |u|_{inf}<u_{max}
%ux = (base_grad(2)>=0)*evader.uMax + (base_grad(2)<0)*evader.uMin;
%uy = (base_grad(4)>=0)*evader.uMax + (base_grad(4)<0)*evader.uMin;

% Controller2: |u|_{2}<u_{max} optimal
normalizer = norm([base_grad(2), base_grad(4)]) + eps;
ux = (base_grad(2))*evader.uMax/normalizer;
uy = (base_grad(4))*evader.uMax/normalizer;
uo = [ux; uy];

min_h = base_grad(1)*base_x(2)+base_grad(3)*base_x(4) + ... 
    -2*(base_grad(2)*ux + base_grad(4)*uy);
max_h = base_grad(1)*base_x(2)+base_grad(3)*base_x(4);

if min_h * max_h < 0
    %dV/dt
    dt = 0.05; %Change it to tm.dV
    dV_dt = -(data(:,:,:,:,end) - data(:,:,:,:,end-1))/dt; % check the order because time is going backwards
    base_dV_dt = eval_u(g, dV_dt, base_x);
    % Controller3: |u|_{2}<u_{max} left minimum
    M = -base_grad(2)/(base_grad(4)+ eps);
    B = (-base_dV_dt - base_grad(1)*base_x(2)-base_grad(3)*base_x(4)+...
        norm([base_grad(2),base_grad(4)])*pursuer.uMax)/(base_grad(4)+eps);
    a = 1 + M^2;
    b = 2*M*B;
    c = B^2 - evader.uMax^2;
    % Controller 4a (+):
    ux = (-b + sqrt(b^2-4*a*c))/(2*a);
    uy = M * ux + B;
    ur = [ux; uy];
    % Controller 4b (-):
    ux = (-b - sqrt(b^2-4*a*c))/(2*a);
    uy = M * ux + B;
    if abs(imag(ux))>0.01 || abs(imag(uy))>0.01
        error('Complex force is not allowed');
    end
    ul = [ux; uy];
else
    ur = uo;
    ul = uo;
end
% Rotate the control to correspond with the actual heading of the "pursuer"
% Optimal controller is the one that maximize the hamiltonian
uSafeOptimal = rotate2D(uo, theta);
% Rigth and left intercepts for zero hamiltonian an force constrain*
uSafeRight = rotate2D(ur, theta);
uSafeLeft = rotate2D(ul, theta);

end
