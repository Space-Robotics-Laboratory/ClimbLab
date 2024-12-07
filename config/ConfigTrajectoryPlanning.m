classdef ConfigTrajectoryPlanning
  %% Properties
  properties (SetAccess = private, GetAccess = public)
    base_trajectory_type (1, 1) string = "5th_order_bezier";
    limb_trajectory_type (1, 1) string = "7th_order_bezier";
      % "7th_order_bezier", "7th_order_spline"

    % Visualization settings
    visualize_limb_trajectory (1, 1) logical = true;  % true/false
      limb_trajectory_line_style (1, 1) string = ":";
      limb_trajectory_color = [0.5, 0.5, 0.5];
      limb_trajectory_width (1, 1) double = 3;
  end

  %% Constructor
  methods (Access = public)

    function config_trajectory_planning = ConfigTrajectoryPlanning(config)
    % ConfigTrajectoryPlanning() Constructor
    %   Override properties value based on specified config file if config is not "default"
      arguments (Input)
        config (1, 1) {mustBeA(config, "string")};
      end

      if (config == "default")
        return;
      end

      kConfigFileName = "config_" + config;
      kPathToConfigFile = "config" + filesep + "preset" + filesep + kConfigFileName + ".m";
      if (~isfile(kPathToConfigFile))
        error("ERROR: The specified config file does NOT exist.");
      end

      kConfigFile = str2func(kConfigFileName);
      kUserConfig = feval(kConfigFile);

      kDefaultConfigPropName = properties(config_trajectory_planning);

      meta_class = metaclass(kUserConfig);
      meta_props = meta_class.PropertyList;

      for i = 1 : length(meta_props)
        get_access_authorization = meta_props(i, 1).GetAccess{1, 1}.Name;

        if (strcmp(get_access_authorization, "ConfigTrajectoryPlanning"))
          kUserConfigPropName = meta_props(i, 1).Name;

          if (~any(strcmp(kDefaultConfigPropName, kUserConfigPropName)))
            error("ERROR: Invalid property name is specified in user customized config file. " + ...
              "That property name is """ + kUserConfigPropName + """. " + ...
              "Property name defined in user customized config file have to match " + ...
              "default config property name.");
          end

          config_trajectory_planning.(kUserConfigPropName) = kUserConfig.(kUserConfigPropName);
        end
      end
    end

  end

  %% Getter
  methods (Access = public)
    function [base_trajectory_type, limb_trajectory_type] = getTrajectoryType(config_trajectory_planning)
      base_trajectory_type = config_trajectory_planning.base_trajectory_type;
      limb_trajectory_type = config_trajectory_planning.limb_trajectory_type;
    end
    function boolean = getVisualizeLimbTrajectory(config_trajectory_planning)
      boolean = config_trajectory_planning.visualize_limb_trajectory;
    end
    function [line_style, color, width] = getLimbTrajectoryVisualSettings(config_trajectory_planning)
      line_style = config_trajectory_planning.limb_trajectory_line_style;
      color = config_trajectory_planning.limb_trajectory_color;
      width = config_trajectory_planning.limb_trajectory_width;
    end
  end

end  % ConfigTrajectoryPlanning
