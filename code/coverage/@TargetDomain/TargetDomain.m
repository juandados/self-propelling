classdef TargetDomain
  % Highway class (inherits linpath class)
  
  properties
      vertX % Vertices X coordinates
      vertY % Vertices Y coordinates
      polyshape % Polygon as polyshape
      area % Polygon's area
      h % plot handle
  end
  
  methods
    function obj = TargetDomain(vertX, vertY)

      obj.vertX = vertX;
      obj.vertY = vertY;
      obj.polyshape = polyshape(vertX, vertY);
      obj.area = polyarea(vertX, vertY);
      
    end
  end
end

