classdef ConfigTrajectoryPlanning < Configuration

  %% Properties
  properties (SetAccess = {?ConfigTrajectoryPlanning, ?Configuration}, GetAccess = public)
    % Base CoM trajectory type
    base_trajectory_type (1, 1) string = "5th_order_bezier";
    % Limb end-effector trajectory type
    % "7th_order_bezier", "7th_order_spline"
    limb_trajectory_type (1, 1) string = "7th_order_bezier";

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

      config_trajectory_planning = config_trajectory_planning.override(config);
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
