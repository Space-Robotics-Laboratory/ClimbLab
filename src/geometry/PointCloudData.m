classdef PointCloudData

  properties (SetAccess = immutable, GetAccess = public)
    x (1, :) double;
    y (1, :) double;
    z (1, :) double;
  end

  methods (Access = public)
    % Constructor
    function this = PointCloudData(x_pos, y_pos, z_pos)
      if     ~all(size(x_pos) == size(y_pos)) ...
          || ~all(size(x_pos) == size(z_pos)) ...
          || ~all(size(y_pos) == size(z_pos))
        error("Inputs (x, y, and z) should be 1xn vectors with same size!!");
      end
      this.x = x_pos;
      this.y = y_pos;
      this.z = z_pos;
    end
  end

end
% EOF