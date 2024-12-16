classdef TrajectoryPlanning < handle
% TrajectoryPlanning
% Plan robot base and limb end-effector pose trajectories and calculate these desired pose at current time step
%
% Created     : 2020.04.10 by Warley Ribeiro
% Last updated: 2024.12.12 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    base_trajectory_ BaseTrajectory;
    limb_trajectory_ LimbTrajectory;
  end

  %% Public Methods
  methods (Access = public)

    function trajectory_planning = TrajectoryPlanning(config_trajectory_planning, robot)
    % TrajectoryPlanning() Constructor
      arguments (Input)
        config_trajectory_planning (1, 1) {mustBeA(config_trajectory_planning, "ConfigTrajectoryPlanning")};
        robot  (1, 1) {mustBeA(robot, "Robot")};
      end

      trajectory_planning.base_trajectory_ = BaseTrajectory(config_trajectory_planning);

      for limb_id = 1 : robot.LP_.getNumberOfLimb()
        trajectory_planning.limb_trajectory_(limb_id, 1) = LimbTrajectory(config_trajectory_planning);
      end
    end

    function plan(trajectory_planning, current_time, robot, foothold_planning, gait_planning)
    % plan()
    %   Plan trajectories from current to desired pose of base and End-Effectors, and
    %   update desired pose for the current time step of both
      arguments (Input)
        trajectory_planning;
        current_time      (1, 1) {mustBeA(current_time,      "double")};
        robot             (1, 1) {mustBeA(robot,             "Robot")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
        gait_planning     (1, 1) {mustBeA(gait_planning,     "GaitPlanning")};
      end

      swing_limb_id = foothold_planning.planner_.output_.getSwingLimbId();

      swing_time = gait_planning.scheduler_.output_.getSwingTimings();
      landing_time = gait_planning.scheduler_.output_.getLandingTimings();

      if (current_time == 0.0 || any(current_time == swing_time(1, swing_limb_id)))
        trajectory_planning.planTrajectories(robot, foothold_planning, gait_planning);
      end

      contact_EE_positions = robot.SV_.contact_state_.getPosition();

      motion_start_time = swing_time(1, swing_limb_id);
      motion_final_time = landing_time(1, swing_limb_id);

      trajectory_planning.updateForCurrentTimeStep( ...
        current_time, contact_EE_positions, swing_limb_id, motion_start_time, motion_final_time);
    end

    function visualize(trajectory_planning, time)
    % visualize()
    %   Visualize trajectories of each end-effector
      arguments (Input)
        trajectory_planning;
        time (1, 1) {mustBeA(time, "double")};
      end

      if (time ~= 0.0)
        return;
      end

      kNumLimb = length(trajectory_planning.limb_trajectory_);

      for limb_id = 1 : kNumLimb
        trajectory_planning.limb_trajectory_(limb_id, 1).position_.planned_trajectory_.visualize();
      end
    end

  end

  %% Private Methods
  methods (Access = private)

    function planTrajectories(trajectory_planning, robot, foothold_planning, gait_planning)
      arguments (Input)
        trajectory_planning;
        robot             (1, 1) {mustBeA(robot,             "Robot")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
        gait_planning     (1, 1) {mustBeA(gait_planning,     "GaitPlanning")};
      end

      trajectory_planning.base_trajectory_.plan(robot, gait_planning);

      kNumLimb = uint8(size(trajectory_planning.limb_trajectory_, 1));
      swing_limb_id = foothold_planning.planner_.output_.getSwingLimbId();

      for limb_id = 1 : kNumLimb
        if (all(limb_id ~= swing_limb_id))
          continue;
        end
        trajectory_planning.limb_trajectory_(limb_id, 1).plan(robot, foothold_planning, gait_planning, limb_id);
      end
    end

    function updateForCurrentTimeStep(trajectory_planning, ...
        current_time, contact_EE_positions, swing_limb_id, motion_start_time, motion_final_time)
      arguments (Input)
        trajectory_planning;
        current_time         (1, 1) {mustBeA(current_time,         "double")};
        contact_EE_positions (3, :) {mustBeA(contact_EE_positions, "double")};
        swing_limb_id        (:, 1) {mustBeA(swing_limb_id,        "uint8")};
        motion_start_time    (1, 1) {mustBeA(motion_start_time,    "double")};
        motion_final_time    (1, 1) {mustBeA(motion_final_time,    "double")};
      end

      trajectory_planning.base_trajectory_.update(current_time, motion_start_time, motion_final_time);

      kNumLimb = size(trajectory_planning.limb_trajectory_, 1);

      for limb_id = 1 : kNumLimb
        if (any(limb_id == swing_limb_id))
          trajectory_planning.limb_trajectory_(limb_id, 1).update(current_time, motion_start_time, motion_final_time);
        else
          trajectory_planning.limb_trajectory_(limb_id, 1).stay(contact_EE_positions(:, limb_id));
        end
      end
    end

  end

  %% Getter
  methods (Access = public)

    function desired_base_position = getDesiredBasePosition(trajectory_planning)
      desired_base_position = trajectory_planning.base_trajectory_.position_.getDesiredPosition();
    end

    function desired_EE_positions = getDesiredEEPositions(trajectory_planning)
      kNumLimb = length(trajectory_planning.limb_trajectory_);
      desired_EE_positions = zeros(3, kNumLimb);
      for limb_id = 1 : kNumLimb
        desired_EE_positions(:, limb_id) = ...
          trajectory_planning.limb_trajectory_(limb_id, 1).position_.getDesiredPosition();
      end
    end

  end

end  % TrajectoryPlanning
