classdef MotionPlanning
% MotionPlanning
% Plan robot base and limb end-effector pose trajectories and calculate these desired pose at current time step
%
% Created     : 2020.04.10 by Warley Ribeiro
% Last updated: 2024.12.07 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    kBaseTrajectoryType_ (1, 1) string;
    kLimbTrajectoryType_ (1, 1) string;
  end
  properties (SetAccess = private, GetAccess = public)
    base_trajectory_ BaseTrajectory;
    limb_trajectory_ LimbTrajectory;
  end

  %% Public Methods
  methods (Access = public)

    function motion_planning = MotionPlanning(config_motion_planning, robot)
    % MotionPlanning() Constructor
      arguments (Input)
        config_motion_planning (1, 1) {mustBeA(config_motion_planning, "ConfigMotionPlanning")};
        robot  (1, 1) {mustBeA(robot, "Robot")};
      end

      num_limb = robot.LP.getNumberOfLimb();
      [motion_planning.kBaseTrajectoryType_, motion_planning.kLimbTrajectoryType_] = ...
        config_motion_planning.getTrajectoryType();

      motion_planning.base_trajectory_ = BaseTrajectory(config_motion_planning);

      for limb_id = 1 : num_limb
        motion_planning.limb_trajectory_(limb_id, 1) = LimbTrajectory(config_motion_planning);
      end
    end

    function motion_planning = plan(motion_planning, ...
        current_time, robot, foothold_planning, gait_planning)
    % plan()
    %   Plan trajectories from current to desired pose of base and End-Effectors, and
    %   update desired pose for the current time step of both
      arguments (Input)
        motion_planning;
        current_time      (1, 1) {mustBeA(current_time, "double")};
        robot             (1, 1) {mustBeA(robot, "Robot")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
        gait_planning     (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
      end

      swing_limb_id = foothold_planning.getSwingLimbID();
      swing_time = gait_planning.getSwingTimings();
      landing_time = gait_planning.getLandingTimings();
      if (current_time == 0.0 || any(current_time == swing_time(1, swing_limb_id)))
        motion_planning = motion_planning.planTrajectories(robot, foothold_planning, gait_planning);
      end

      contact_EE_positions = robot.contact_state.getPosition();
      motion_start_time = swing_time(1, swing_limb_id);
      motion_final_time = landing_time(1, swing_limb_id);

      motion_planning = motion_planning.updateForCurrentTimeStep( ...
        current_time, contact_EE_positions, swing_limb_id, motion_start_time, motion_final_time);
    end

    function motion_planning = visualize(motion_planning, time)
    % visualize()
    %   Visualize trajectories of each end-effector
      arguments (Input)
        motion_planning;
        time (1, 1) {mustBeA(time, "double")};
      end

      if (time ~= 0.0)
        return;
      end
      kNumLimb = length(motion_planning.limb_trajectory_);
      for limb_id = 1 : kNumLimb
        motion_planning.limb_trajectory_(limb_id, 1).position_.planned_trajectory_.visualize();
      end
    end

  end

  %% Private Methods
  methods (Access = private)

    function motion_planning = planTrajectories(motion_planning, ...
        robot, foothold_planning, gait_planning)
      arguments (Input)
        motion_planning;
        robot             (1, 1) {mustBeA(robot, "Robot")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
        gait_planning     (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
      end

      motion_planning.base_trajectory_ = motion_planning.base_trajectory_.plan(robot, gait_planning);

      kNumLimb = uint8(size(motion_planning.limb_trajectory_, 1));
      swing_limb_id = foothold_planning.getSwingLimbID();
      for limb_id = 1 : kNumLimb
        if (all(limb_id ~= swing_limb_id))
          continue;
        end
        motion_planning.limb_trajectory_(limb_id, 1) = ...
          motion_planning.limb_trajectory_(limb_id, 1).plan( ...
            robot, foothold_planning, gait_planning, limb_id);
      end
    end

    function motion_planning = updateForCurrentTimeStep(motion_planning, ...
        current_time, contact_EE_positions, swing_limb_id, motion_start_time, motion_final_time)
      arguments (Input)
        motion_planning;
        current_time         (1, 1) {mustBeA(current_time, "double")};
        contact_EE_positions (3, :) {mustBeA(contact_EE_positions, "double")};
        swing_limb_id        (:, 1) {mustBeA(swing_limb_id, "uint8")};
        motion_start_time    (1, 1) {mustBeA(motion_start_time, "double")};
        motion_final_time    (1, 1) {mustBeA(motion_final_time, "double")};
      end

      motion_planning.base_trajectory_ = motion_planning.base_trajectory_.update( ...
        current_time, motion_start_time, motion_final_time);

      kNumLimb = size(motion_planning.limb_trajectory_, 1);
      for limb_id = 1 : kNumLimb
        if (any(limb_id == swing_limb_id))
          motion_planning.limb_trajectory_(limb_id, 1) = ...
            motion_planning.limb_trajectory_(limb_id, 1).update(current_time, motion_start_time, ...
            motion_final_time);
        else
          motion_planning.limb_trajectory_(limb_id, 1) = ...
            motion_planning.limb_trajectory_(limb_id, 1).stay(contact_EE_positions(:, limb_id));
        end
      end
    end

  end

  %% Getter
  methods (Access = public)
    function desired_base_position = getDesiredBasePosition(motion_planning)
      desired_base_position = motion_planning.base_trajectory_.position_.getDesiredPosition();
    end
    function desired_EE_positions = getDesiredEEPositions(motion_planning)
      kNumLimb = length(motion_planning.limb_trajectory_);
      desired_EE_positions = zeros(3, kNumLimb);
      for limb_id = 1 : kNumLimb
        desired_EE_positions(:, limb_id) = ...
          motion_planning.limb_trajectory_(limb_id, 1).position_.getDesiredPosition();
      end
    end
  end

end  % MotionPlanning
