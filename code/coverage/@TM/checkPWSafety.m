function [safe, uSafe, safe_val] = checkPWSafety(obj, i, j)
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
  uSafe = [];
  safe_val = 10;
  return
end

switch(class(obj.aas{i}))
  case 'UTMQuad4D'
    %% agent i is a quadrotor
    switch(class(obj.aas{j}))
      case 'UTMQuad4D'
        [safe, uSafe, safe_val] = checkPWSafety_qr_qr(obj.qr_qr_safe_V, ...
          obj.safetyTime, obj.aas{i}, obj.aas{j});
        
      otherwise
        error('Unknown agent type')
        
    end % end inner switch
  otherwise
    error('Unknown agent type')
end % end outer switch

end % end function

%%
function [safe, uSafe, valuex] = checkPWSafety_qr_qr(qr_qr_safe_V, safetyTime, ...
  evader, pursuer)
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
  uSafe = [];
  valuex = 10;
  return
end

% Compute safety value
valuex = eval_u(qr_qr_safe_V.g, qr_qr_safe_V.data, base_x);

% Compute safety preserving control if needed
if valuex > safetyTime %%|| rand > 0.98
  % Safe; no need to worry about computing controller
  safe = 1;
  uSafe = [];
  return
end

% Not safe, compute safety controller
safe = 0;

% Compute control assuming "pursuer" is facing 0 degrees
% base_grad = calculateCostate(qr_qr_safe_V.g, qr_qr_safe_V.grad, base_x);
% In the help of calculateCostate it says that it is not needed anymore, because it could be replaced by eval_u
base_grad = eval_u(qr_qr_safe_V.g, qr_qr_safe_V.grad, base_x);

% Controller1: |u|_{inf}<u_{max}
%ux = (base_grad(2)>=0)*evader.uMax + (base_grad(2)<0)*evader.uMin;
%uy = (base_grad(4)>=0)*evader.uMax + (base_grad(4)<0)*evader.uMin;

% Controller2: |u|_{2}<u_{max} maximum
normalizer = norm([base_grad(2), base_grad(4)]) + eps;
ux = (base_grad(2))*evader.uMax/normalizer;
uy = (base_grad(4))*evader.uMax/normalizer;

% Controller3: |u|_{2}<u_{max} left minimum
M = -base_grad(2)/(base_grad(4)+ eps);
B = (-base_grad(1)*base_x(1)-base_grad(3)*base_x(3)+...
    norm([base_grad(2),base_grad(4)])*pursuer.uMax)/(base_grad(4)+eps);
if safe ~= true
    
    xx = [-pursuer.uMax:0.05:pursuer.uMax];
    yy = M*xx+B;
    disp('m');
    disp(M);
    disp('B');
    disp(B);
    figure(100);
    plot(xx, yy);
    hold on;
    th = 0:0.1:2*pi;
    plot(pursuer.uMax*cos(th),pursuer.uMax*sin(th));
    hold off;
    axis([-pursuer.uMax, pursuer.uMax, -50*pursuer.uMax, 50*pursuer.uMax]);
end
% a = 1+(base_grad(2)/base_grad(4))^2;
% b = -2*(base_grad(2)/base_grad(4)^2)*(-base_grad(1)*base_x(1)- ...
%     base_grad(3)*base_x(3)+norm([base_grad(2),base_grad(4)])*pursuer.uMax);
% c = ((-base_grad(1)*base_x(1)-base_grad(3)*base_x(3)+ ...
%     norm([base_grad(2),base_grad(4)])*pursuer.uMax)/base_grad(4))^2-evader.uMax^2;
% Controller 4a (+):
% ux = (-b + sqrt(b^2-4*a*c))/(2*a);
% uy = M * ux + B;
% disp('ux');
% disp(ux);
% disp('force');
% disp(sqrt(ux^2+uy^2));

% Controller 4b (-):
% ux = (-b - sqrt(b^2-4*a*c))/(2*a);
% uy = M * x2 + B;

u = [ux; uy];


% Rotate the control to correspond with the actual heading of the
% "pursuer"
uSafe = rotate2D(u, theta);  

end
