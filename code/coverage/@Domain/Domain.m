classdef Domain
  % Highway class (inherits linpath class)
  
  properties
    % width of highway
    width

    % highway connections/segways
    connections
    % CURRENTLY:
    % n by 1 vector where the ith element is the s parameter on the current
    % highway where it intersects with the ith highway
    %
    % ALTERNATIVE OPTION FOR THE FUTURE:
    % n by 2 cell structure
    % {s0, i/o, {hw0, s_onhw0};
    %  s1, i/o, {hw1, s_onhw1};
    %  ... }
    
    ps % platoons on the highway
  end
  
  methods
    function obj = Highway(z0, z1, speed, width)
      % function obj = highway(z0, z1, speed, width)
      % Constructor for highway class
      %
      % Inputs:  z0     - starting point (in 2D)
      %          z1     - ending point
      %          speed  - speed of travel
      %          width  - width of highway
      %
      % Output:  obj    - highway object
      %
      % Mo Chen, 2015-05-25
      % Modified: 2015-07-21

      %% Default speed and width
      if nargin < 3
        speed = 10;
      end
      
      if nargin < 4
        width = 10;
      end
      
      % Call constructor of the superclass linpath
      obj@Linpath(z0, z1, speed);
      
      % width of highway
      obj.width = width;
      
      warning('Need to specify connections property')
    end
  end
end

