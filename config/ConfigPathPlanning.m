classdef ConfigPathPlanning
  %% Properties
  properties (SetAccess = private, GetAccess = public)
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

      config_path_planning = config_path_planning.override(config);

      config_path_planning = config_path_planning.isValidPathPlanningMethod();
    end

  end

  %% Private Methods
  methods (Access = private)

    function config_path_planning = override(config_path_planning, config)
    % override()
    %   Override properties value based on specified config file if config is not "default"
      if (config == "default")
        return;
      end

      config_file_name = "config_" + config;
      if (~isfile("config\preset\" + config_file_name + ".m"))
        error("ERROR: The specified config file does NOT exist.");
      end

      config_file = str2func(config_file_name);
      user_config = feval(config_file);

      this_config_prop_name = properties(config_path_planning);

      meta_class = metaclass(user_config);
      meta_props = meta_class.PropertyList;

      for i = 1 : length(meta_props)
        get_access_authorization = meta_props(i, 1).GetAccess{1, 1}.Name;

        if (strcmp(get_access_authorization, "ConfigPathPlanning"))
          user_config_prop_name = meta_props(i, 1).Name;

          if (~any(strcmp(this_config_prop_name, user_config_prop_name)))
            error("ERROR: Invalid property name is specified in user customized config file. " + ...
              "That property name is """ + user_config_prop_name + """. " + ...
              "Property name defined in user customized config file have to match " + ...
              "default config property name.");
          end

          config_path_planning.(user_config_prop_name) = user_config.(user_config_prop_name);
        end
      end
    end

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
end
% EOF