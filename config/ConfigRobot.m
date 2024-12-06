classdef ConfigRobot
  % Configure robot parameters
  %% Properties
  properties (SetAccess = private, GetAccess = public)
    % "HubRobo_v3_2_grip_to_palm"
    robot_type (1, 1) string = "HubRobo_v3_2_grip_to_palm";

    % Initial robot base position in Ground frame
    initial_base_position (3, 1) double = [0.0; 0.0; 0.07];  % [m]
    % Initial robot base orientation (euler) in Ground frame
    initial_base_orientation_euler (3, 1) double = [0.0; 0.0; 0.0];  % [deg]

    % Desired initial End-Effector (x, y) distance from robot base CoM in Ground frame
    desired_initial_EE_distance_xy_from_base_CoM (2, 1) double = [0.15; 0.15];  % [m]

    % "max_holding_force"
    gripper_detachment_detection_method (1, 1) string = "max_holding_force";


    % Visualization settings
    visualize_robot (1, 1) logical = true;
      base_upper_thickness (1, 1) double = 0.03;  % [m]
      base_lower_thickness (1, 1) double = 0.03;  % [m]
      link_radius (1, 1) double = 0.012;  % [m]
      base_color (3, 1) double = [0.2, 0.2, 0.2];
      limb_color (3, 1) double = [0.2, 0.2, 0.2];
      base_alpha (1, 1) double = 0.8;  % [0, 1]
      limb_alpha (1, 1) double = 0.8;  % [0, 1]
  end

  %% Constructor
  methods (Access = public)

    function config_robot = ConfigRobot(config)
    % ConfigRobot() Constructor
    %   Override properties value based on specified config file if config is not "default"
      arguments (Input)
        config (1, 1) {mustBeA(config, "string")};
      end
      if (config == "default")
        return;
      end

      config_file_name = "config_" + config;
      if (~isfile("config\preset\" + config_file_name + ".m"))
        error("ERROR: The specified config file does NOT exist.");
      end

      config_file = str2func(config_file_name);
      user_config = feval(config_file);

      this_config_prop_name = properties(config_robot);

      meta_class = metaclass(user_config);
      meta_props = meta_class.PropertyList;

      for i = 1 : length(meta_props)
        get_access_authorization = meta_props(i, 1).GetAccess{1, 1}.Name;

        if (strcmp(get_access_authorization, "ConfigRobot"))
          user_config_prop_name = meta_props(i, 1).Name;

          if (~any(strcmp(this_config_prop_name, user_config_prop_name)))
            error("ERROR: Invalid property name is specified in user customized config file. " + ...
              "That property name is """ + user_config_prop_name + """. " + ...
              "Property name defined in user customized config file have to match " + ...
              "default config property name.");
          end

          config_robot.(user_config_prop_name) = user_config.(user_config_prop_name);
        end
      end
    end

  end

  %% Getter
  methods (Access = public)
    function robot_type = getRobotType(config_robot)
      robot_type = config_robot.robot_type;
    end
    function initial_base_position = getInitialBasePosition(config_robot)
      initial_base_position = config_robot.initial_base_position;
    end
    function initial_base_orientation_euler = getInitialBaseOrientationDCM(config_robot)
      initial_base_orientation_euler = config_robot.initial_base_orientation_euler;
    end
    function desired_initial_EE_distance_xy_from_base_CoM = ...
        getInitialEEDistXYFromBaseCoM(config_robot)
      desired_initial_EE_distance_xy_from_base_CoM = ...
        config_robot.desired_initial_EE_distance_xy_from_base_CoM;
    end
    function gripper_detachment_detection_method = getGripperDetachmentDetectionMethod(config_robot)
      gripper_detachment_detection_method = config_robot.gripper_detachment_detection_method;
    end

    function boolean = getVisualizeRobot(config_robot)
      boolean = config_robot.visualize_robot;
    end
    function [base_upper_thickness, base_lower_thickness, base_color, base_alpha] = ...
        getBaseVisualSettings(config_robot)
      base_upper_thickness = config_robot.base_upper_thickness;
      base_lower_thickness = config_robot.base_lower_thickness;
      base_color = config_robot.base_color;
      base_alpha = config_robot.base_alpha;
    end
    function [link_radius, limb_color, limb_alpha] = getLimbVisualSettings(config_robot)
      link_radius = config_robot.link_radius;
      limb_color = config_robot.limb_color;
      limb_alpha = config_robot.limb_alpha;
    end
  end

end
% EOF