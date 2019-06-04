function addPlatoon(obj, platoon)
% addPlatoon(obj, platoon)
% Adds a platoon to the highway object

obj.ps{length(obj.ps) + 1, 1} = platoon;
platoon.hw = obj;

end