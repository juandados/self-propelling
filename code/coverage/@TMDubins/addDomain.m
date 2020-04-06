function addDomain(obj, domain)
% function addHighway(obj, hw)
%
% Adds a highway to the list of highways managed by the tfm
%
% Mo Chen, 2015-10-30

if ~isa(domain, 'TargetDomain')
  error('Input must be a domain object!')
end

% Add highway to highways list
obj.domain = domain;

end