classdef ConfigPathPlanning < Configuration

  %% Properties
  properties (SetAccess = {?ConfigPathPlanning, ?Configuration}, GetAccess = public)
    goal_position (3, 1) double = [1.0; 0.0; 0.0];  % [m]

    % Global Path Planning method
    % ("do_nothing", "straight_toward_the_goal_direction")
    global_path_plan_type (1, 1) string = "straight_toward_the_goal_direction";

    % Local Path Planning method
    % ("do_nothing", "LPP_based_on_next_way_point")
    local_path_plan_type  (1, 1) string = "LPP_based_on_next_way_point";
  end

  %% Constructor
  methods (Access = public)

    function config_path_planning = ConfigPathPlanning(config)
    % ConfigPathPlanning() Constructor
      arguments (Input)
        config (1, 1) {mustBeA(config, "string")};
      end

      if (config == "default")
        return;
      end

      config_path_planning = config_path_planning.override(config);

      % TODO: This should be delete
      config_path_planning = config_path_planning.isValidPathPlanningMethod();
    end

  end

  %% Private Methods
  methods (Access = private)

    function config_path_planning = isValidPathPlanningMethod(config_path_planning)
      if (config_path_planning.global_path_plan_type ~= "do_nothing" && ...
          config_path_planning.local_path_plan_type == "do_nothing")
        error("ERROR: Specification of global and local path planning method is invalid. " + ...
          "If ""global_path_planning_type"" is not ""do_nothing"", " + ...
          """local_path_planning_type"" should not be ""do_nothing"".")
      end
    end

  end

  %% Getter
  methods (Access = public)
    function goal_position = getGoalPosition(config_path_planning)
      goal_position = config_path_planning.goal_position;
    end
    function global_path_plan_type = getGlobalPathPlanningType(config_path_planning)
      global_path_plan_type = config_path_planning.global_path_plan_type;
    end
    function local_path_plan_type = getLocalPathPlanningType(config_path_planning)
      local_path_plan_type = config_path_planning.local_path_plan_type;
    end
  end
end  % ConfigPathPlanning
