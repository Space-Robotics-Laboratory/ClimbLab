classdef GaitPlanning
% GaitPlanning
% Plan desired base pose and gait schedule
%
% Created     : 2021.04.08 by Warley Ribeiro
% Last updated: 2024.10.22 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    type (1, 1) string;
  end
  properties (SetAccess = private, GetAccess = public)
    base_pose_planner;
  end
  properties (SetAccess = private, GetAccess = public)
    scheduler;

    swing_timings   double;  % [s]: Timing of limb transfer motion start for each limb
    landing_timings double;  % [s]: Timing of limb transfer motion end for each limb

    transfer_duration (1, 1) double;  % [s] including gripper release, limb swing, gripper grasp
    swing_duration    (1, 1) double;  % [s]
    release_duration  (1, 1) double;  % [s]
    grasp_duration    (1, 1) double;  % [s]

    support_duration          (1, 1) double;  % [s]
    all_limb_support_duration (1, 1) double;  % [s]
  end

  %% Public Methods
  methods (Access = public)

    function gait_planning = GaitPlanning(config)
    % GaitPlanning() Constructor
      arguments (Input)
        config (1, 1) {mustBeA(config, "ConfigGaitPlanning")};
      end
      gait_planning.type = config.getGaitType();

      gait_planning.base_pose_planner = BasePosePlanning(config.getBasePosePlaningType());

      gait_planning.scheduler = gait_planning.setScheduler(config);

      gait_planning.support_duration = gait_planning.scheduler.calcSupportDuration();
      gait_planning.transfer_duration = gait_planning.scheduler.calcTransferDuration( ...
        gait_planning.support_duration);
      [gait_planning.release_duration, gait_planning.grasp_duration] = ...
        config.getGripperReleaseAndGraspDuration();
      gait_planning.swing_duration = gait_planning.scheduler.calcSwingDuration(gait_planning);

      gait_planning.all_limb_support_duration = ...
        gait_planning.scheduler.calcAllLimbSupportDuration(gait_planning);

      [gait_planning.swing_timings, gait_planning.landing_timings] = ...
        gait_planning.scheduler.initializeLimbMotionTimings(gait_planning);
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

      swing_limb_id = foothold_planning.getSwingLimbID();
      if (~gait_planning.isUpdateTiming(current_time, swing_limb_id))
        return;
      end

      gait_planning.base_pose_planner = gait_planning.base_pose_planner.plan( ...
        robot, path_planning, foothold_planning);

      [gait_planning.swing_timings, gait_planning.landing_timings] = ...
        gait_planning.scheduler.updateSwingAndLandingTiming( ...
        current_time, gait_planning, swing_limb_id);
    end

  end

  %% Private Methods
  methods (Access = private)
    
    function boolean = isUpdateTiming(gait_planning, current_time, swing_limb_id)
      swing_time = gait_planning.getSwingTimings();
      if (current_time ~= 0.0 && any(current_time ~= swing_time(1, swing_limb_id)))
        boolean = false;
      else
        boolean = true;
      end
    end

  end

  %% Setter
  methods (Access = private)

    function scheduler = setScheduler(gait_planning, config)
      if startsWith(gait_planning.type, "periodic")
        scheduler = PeriodicGait(config);
      elseif startsWith(gait_planning.type, "non_periodic")
        scheduler = NonPeriodicGait();
      else
        error("ERROR: Failed to set gait scheduler. " + ...
          "Gait scheduler type starts with ""periodic"" or ""non_periodic"".");
      end
    end

  end

  %% Getter
  methods (Access = public)
    function type = getType(gait_planning)
      type = gait_planning.type;
    end

    function transfer_duration = getTransferDuration(gait_planning)
      transfer_duration = gait_planning.transfer_duration;
    end
    function release_duration = getReleaseDuration(gait_planning)
      release_duration = gait_planning.release_duration;
    end
    function grasp_duration = getGraspDuration(gait_planning)
      grasp_duration = gait_planning.grasp_duration;
    end
    function all_limb_support_duration = getAllLimbSupportDuration(gait_planning)
      all_limb_support_duration = gait_planning.all_limb_support_duration;
    end
    function swing_timings = getSwingTimings(gait_planning)
      swing_timings = gait_planning.swing_timings;
    end
    function landing_timings = getLandingTimings(gait_planning)
      landing_timings = gait_planning.landing_timings;
    end
  end

end
% EOF