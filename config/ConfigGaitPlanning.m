classdef ConfigGaitPlanning < Configuration

  %% Properties
  properties (SetAccess = {?ConfigGaitPlanning, ?Configuration}, GetAccess = public)
    gait_type (1, 1) string = "periodic_crawl";
      % "periodic_crawl", "periodic_trot"

    % Periodic gait settings
    gait_period (1, 1) double = 4.0;  % [s]
    duty_factor (1, 1) double = 0.75;  % [0, 1]
    sequence uint8 = [2, 1, 3, 4];
      % 1st dim: Limb number(s) starting at same timing during gait cycle
      % 2nd dim: Limb number(s) starting at different timing during gait cycle

    foot_lift_up_duration   (1, 1) double = 0.0;  % [s]
    foot_lift_down_duration (1, 1) double = 0.0;  % [s]
  end
  properties (SetAccess = {?ConfigGaitPlanning, ?Configuration}, GetAccess = public)
    % "do_nothing", "intersection_of_diagonal_lines",
    % "intersection_of_diagonal_line_and_moving_direction"
    base_position_planning_type    (1, 1) string = "intersection_of_diagonal_lines";
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

      config_gait_planning = config_gait_planning.override(config);
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
    function [foot_lift_up_duration, foot_lift_down_duration] = ...
      getFootLiftUpAndDownDuration(config_gait_planning)
      foot_lift_up_duration = config_gait_planning.foot_lift_up_duration;
      foot_lift_down_duration = config_gait_planning.foot_lift_down_duration;
    end

    function base_pose_planning_type = getBasePosePlaningType(config_gait_planning)
      base_pose_planning_type = [ config_gait_planning.base_position_planning_type;
                                  config_gait_planning.base_orientation_planning_type];
    end
  end

end  % ConfigGaitPlanning
