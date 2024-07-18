classdef (Abstract) GlobalPathPlanner

  properties (Abstract, SetAccess = ?GlobalPathPlanner, GetAccess = public)
    moving_direction (3, 1) double;
  end

  methods (Abstract, Access = {?PathPlanning, ?GlobalPathPlanner})
    plan(global_path);
  end

  methods (Access = public)
  end

end
% EOF