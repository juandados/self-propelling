function constrained = constrainState(obj)
% 
% constrained = constrainState(obj)
% Apply the DubinsCarAccelerated states constrains. This constrains are on velocity
%
% Inputs:   obj - current quardotor object
%
% Outputs:  done - indicates if any constrain was applied.
%
% Juan Chacon, 2020-03-03

% Adding velocity contstrain
speed=abs(obj.x(4));

if speed >= obj.speedLimit
  obj.x(4) = obj.speedLimit;
end

if speed < obj.speedMin
  obj.x(4) = obj.speedMin;
end

constrained = true;

end
