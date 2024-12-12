classdef GaitPlanning < handle
% GaitPlanning
% Plan desired base pose and gait schedule
%
% Created     : 2021.04.08 by Warley Ribeiro
% Last updated: 2024.12.12 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    kType_ (1, 1) string;
  end
  properties (SetAccess = private, GetAccess = public)
    base_pose_planner_;
  end
  properties (SetAccess = private, GetAccess = public)
    scheduler_;

    kStepHeight_           (1, 1) double;  % [m]

    kFootLiftUpDuration_   (1, 1) double;  % [s]
    kFootLiftDownDuration_ (1, 1) double;  % [s]
  end

  %% Public Methods
  methods (Access = public)

    function gait_planning = GaitPlanning(config_gait_planning)
    % GaitPlanning() Constructor
      arguments (Input)
        config_gait_planning (1, 1) {mustBeA(config_gait_planning, "ConfigGaitPlanning")};
      end
      gait_planning.kType_ = config_gait_planning.getGaitType();

      gait_planning.kStepHeight_ = config_gait_planning.getStepHeight();

      [gait_planning.kFootLiftUpDuration_, gait_planning.kFootLiftDownDuration_] = ...
        config_gait_planning.getFootLiftUpAndDownDuration();

      gait_planning.base_pose_planner_ = BasePosePlanning(config_gait_planning.getBasePosePlaningType());

      gait_planning.setScheduler(config_gait_planning);

      gait_planning.scheduler_.calcSupportDuration();
      gait_planning.scheduler_.calcTransferDuration();
      gait_planning.scheduler_.calcSwingDuration(gait_planning.kFootLiftUpDuration_, gait_planning.kFootLiftDownDuration_);
      gait_planning.scheduler_.calcAllLimbSupportDuration();

      gait_planning.scheduler_.initializeLimbMotionTimings();
    end

    function gait_planning = plan(gait_planning, ...
        current_time, robot, path_planning, foothold_planning)
    % plan()
    %   Plan the next base pose and the gait schedule
      arguments (Input)
        gait_planning;
        current_time      (1, 1) {mustBeA(current_time, "double")};
        robot             (1, 1) {mustBeA(robot, "Robot")};
        path_planning     (1, 1) {mustBeA(path_planning, "PathPlanning")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
      end

      swing_limb_id = foothold_planning.planner_.output_.getSwingLimbId();
      if (~gait_planning.isUpdateTiming(current_time, swing_limb_id))
        return;
      end

      gait_planning.base_pose_planner_ = gait_planning.base_pose_planner_.plan( ...
        robot, path_planning, foothold_planning);

      gait_planning.scheduler_.updateSwingAndLandingTiming(current_time, swing_limb_id);
    end

  end

  %% Private Methods
  methods (Access = private)

    function is_update_timing = isUpdateTiming(gait_planning, current_time, swing_limb_id)
      swing_time = gait_planning.scheduler_.output_.getSwingTimings();
      if (current_time ~= 0.0 && any(current_time ~= swing_time(1, swing_limb_id)))
        is_update_timing = false;
      else
        is_update_timing = true;
      end
    end

  end

  %% Setter
  methods (Access = private)

    function setScheduler(gait_planning, config_gait_planning)
      if startsWith(gait_planning.kType_, "periodic")
        scheduler = PeriodicGait(config_gait_planning);
      elseif startsWith(gait_planning.kType_, "non_periodic")
        scheduler = NonPeriodicGait();
      else
        error("ERROR: Failed to set gait scheduler. " + ...
          "Gait scheduler type starts with ""periodic"" or ""non_periodic"".");
      end
      gait_planning.scheduler_ = scheduler;
    end

  end

  %% Getter
  methods (Access = public)
    function type = getType(gait_planning)
      type = gait_planning.kType_;
    end

    function step_height = getStepHeight(gait_planning)
      step_height = gait_planning.kStepHeight_;
    end
    function foot_lift_up_duration = getFootLiftUpDuration(gait_planning)
      foot_lift_up_duration = gait_planning.kFootLiftUpDuration_;
    end
    function foot_lift_down_duration = getFootLiftDownDuration(gait_planning)
      foot_lift_down_duration = gait_planning.kFootLiftDownDuration_;
    end
  end

end  % GaitPlanning
