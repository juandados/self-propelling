function constrained = constrainState(obj)
% 
% constrained = constrainState(obj)
% Apply the quad4D states constrains. This constrains are on velocity
%
% Inputs:   obj - current quardotor object
%
% Outputs:  done - indicates if any constrain was applied.
%
% Juan Chacon, 2020-03-03

% Adding velocity contstrain
speed=norm([obj.x(2), obj.x(4)]);
if speed >= obj.speedLimit
  obj.x(2)=obj.speedLimit*obj.x(2)/speed;
  obj.x(4)=obj.speedLimit*obj.x(4)/speed;
end
constrained = true;

end
