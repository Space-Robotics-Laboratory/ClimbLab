classdef FootholdPlanning

  properties (SetAccess = private, GetAccess = public)
    type (1, 1) string;
    planner;
  end

  methods (Access = public)
    % Constructor
    function foothold_planning = FootholdPlanning(foothold_selection_type, num_limb)
      arguments (Input)
        foothold_selection_type (1, 1) {mustBeA(foothold_selection_type, "string")};
        num_limb                (1, 1) {mustBeA(num_limb,                "uint8")};
      end
      foothold_planning.type = foothold_selection_type;
      foothold_planning.planner = foothold_planning.setPlanner(num_limb);
    end

    function foothold_planning = initialize(foothold_planning, ...
        sequence, step_length, current_EE_position)
      foothold_planning.planner = ...
        foothold_planning.planner.initialize(sequence, step_length, current_EE_position);
    end

    function foothold_planning = plan(foothold_planning, ...
        current_time, moving_direction, graspable_points)
      foothold_planning.planner = foothold_planning.planner.plan( ...
        current_time, moving_direction, graspable_points);
    end
  end

  methods (Access = private)
    % Setter
    function planner = setPlanner(foothold_planning, num_limb)
      switch foothold_planning.type
        case "fixed_stride"
          planner = FixedStride(num_limb);
        otherwise
      end
    end
  end

end
% EOF