classdef ConfigGaitPlanning

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    gait_type (1, 1) string = "periodic_crawl";
      % "periodic_crawl", "periodic_trot"

    % Periodic gait settings
    gait_period (1, 1) double = 4.0;  % [s]
    duty_factor (1, 1) double = 0.75;  % [0, 1]
    sequence uint8 = [2, 1, 3, 4];
      % 1st dim: Limb number(s) starting at same timing during gait cycle
      % 2nd dim: Limb number(s) starting at different timing during gait cycle

    gripper_release_duration (1, 1) double = 0.0;  % [s]
    gripper_grasp_duration   (1, 1) double = 0.0;  % [s]
  end
  properties (SetAccess = private, GetAccess = public)
    % "do_nothing", "intersection_of_diagonal_lines",
    % "intersection_of_diagonal_line_and_moving_direction"
    base_position_planning_type (1, 1) string = "intersection_of_diagonal_lines";
    base_orientation_planning_type (1, 1) string = "do_nothing";
  end

  %% Constructor
  methods (Access = public)

    function config_gait_planning = ConfigGaitPlanning(config)
    % ConfigGaitPlanning() Constructor
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

      this_config_prop_name = properties(config_gait_planning);

      meta_class = metaclass(user_config);
      meta_props = meta_class.PropertyList;

      for i = 1 : length(meta_props)
        get_access_authorization = meta_props(i, 1).GetAccess{1, 1}.Name;

        if (strcmp(get_access_authorization, "ConfigGaitPlanning"))
          user_config_prop_name = meta_props(i, 1).Name;

          if (~any(strcmp(this_config_prop_name, user_config_prop_name)))
            error("ERROR: Invalid property name is specified in user customized config file. " + ...
              "That property name is """ + user_config_prop_name + """. " + ...
              "Property name defined in user customized config file have to match " + ...
              "default config property name.");
          end

          config_gait_planning.(user_config_prop_name) = user_config.(user_config_prop_name);
        end
      end
    end

  end

  %% Getter
  methods (Access = public)
    function gait_type = getGaitType(config_gait_planning)
      gait_type = config_gait_planning.gait_type;
    end
    function gait_period = getGaitPeriod(config_gait_planning)
      gait_period = config_gait_planning.gait_period;
    end
    function duty_factor = getDutyFactor(config_gait_planning)
      duty_factor = config_gait_planning.duty_factor;
    end
    function sequence = getGaitSequence(config_gait_planning)
      sequence = config_gait_planning.sequence;
    end
    function [gripper_release_duration, gripper_grasp_duration] = ...
        getGripperReleaseAndGraspDuration(config_gait_planning)
      gripper_release_duration = config_gait_planning.gripper_release_duration;
      gripper_grasp_duration = config_gait_planning.gripper_grasp_duration;
    end

    function base_pose_planning_type = getBasePosePlaningType(config_gait_planning)
      base_pose_planning_type = [ config_gait_planning.base_position_planning_type;
                                  config_gait_planning.base_orientation_planning_type];
    end
  end

end
% EOF