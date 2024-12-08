classdef ConfigFootholdPlanning < Configuration

  %% Properties
  properties (SetAccess = {?ConfigFootholdPlanning, ?Configuration}, GetAccess = public)
    % Foothold selection type
    % ("do_nothing", "fixed_stride")
    foothold_selection_type (1, 1) string = "fixed_stride";

    % For "fixed_stride"
    max_allowable_stride (1, 1) double = 0.05;  % [m]
    step_height (1, 1) double = 0.025;  % [m]
  end

  %% Constructor
  methods (Access = public)

    function config_foothold_planning = ConfigFootholdPlanning(config)
    % ConfigFootholdPlanning() Constructor
    %   Override properties value based on specified config file if config is not "default"
      arguments (Input)
        config (1, 1) {mustBeA(config, "string")};
      end

      if (config == "default")
        return;
      end


      config_foothold_planning = config_foothold_planning.override(config);

      % TODO: This should be delete
      if (config_foothold_planning.foothold_selection_type ~= "fixed_stride")
        config_foothold_planning.max_allowable_stride = NaN;
      end
    end

  end

  %% Getter
  methods (Access = public)
    function foothold_selection_type = getFootholdSelectionType(config_foothold_planning)
      foothold_selection_type = config_foothold_planning.foothold_selection_type;
    end
    function max_allowable_stride = getMaxAllowableStride(config_foothold_planning)
      max_allowable_stride = config_foothold_planning.max_allowable_stride;
    end
    function step_height = getStepHeight(config_foothold_planning)
      step_height = config_foothold_planning.step_height;
    end
  end
end  % ConfigFootholdPlanning
