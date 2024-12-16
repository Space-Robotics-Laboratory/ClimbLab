classdef BaseTrajectory < handle
% BaseTrajectory
% Plan robot base pose trajectory and calculate desired pose at current time step
%
% Created     : 2024.05.20 by Masazumi Imai
% Last updated: 2024.12.12 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    kType_ (1, 1) string;
  end
  properties (SetAccess = private, GetAccess = public)
    position_ PositionTrajectory;
    orientation_;
  end
  properties (Constant, GetAccess = private)
    kStartTime_         (1, 1) double = 0.0;              % [s]
    kStartVelocity_     (3, 1) double = [0.0; 0.0; 0.0];  % [m/s]
    kFinalVelocity_     (3, 1) double = [0.0; 0.0; 0.0];  % [m/s]
    kStartAcceleration_ (3, 1) double = [0.0; 0.0; 0.0];  % [m/s^2]
    kFinalAcceleration_ (3, 1) double = [0.0; 0.0; 0.0];  % [m/s^s]
  end

  %% Methods called only from TrajectoryPlanning
  methods (Access = ?TrajectoryPlanning)

    % Constructor
    function base_trajectory = BaseTrajectory(config_trajectory_planning)
      arguments (Input)
        config_trajectory_planning (1, 1) {mustBeA(config_trajectory_planning, "ConfigTrajectoryPlanning")};
      end
      [base_trajectory.kType_, ~] = config_trajectory_planning.getTrajectoryType();

      base_trajectory.position_ = PositionTrajectory(base_trajectory.kType_);

      kLineStyle = "none"; kColor = [0.0, 0.0, 0.0]; kWidth = 0.0;
      base_trajectory.position_.planned_trajectory_.setVisualSettings(kLineStyle, kColor, kWidth);
    end

    function plan(base_trajectory, robot, gait_planning)
    % plan()
    %   Plan the trajectory from current to desired pose of the robot base
      arguments (Input)
        base_trajectory;
        robot         (1, 1) {mustBeA(robot, "Robot")};
        gait_planning (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
      end

      current_base_position = robot.SV_.getBasePosition();  % previous desired position?
      desired_base_position = gait_planning.base_pose_planner_.getDesiredBasePosition();

      motion_duration = gait_planning.scheduler_.output_.getTransferDuration();

      time_constraints = [base_trajectory.kStartTime_, ...
                          motion_duration];

      position_constraints = [current_base_position, ...
                              desired_base_position];

      velocity_constraints = [base_trajectory.kStartVelocity_,...
                              base_trajectory.kFinalVelocity_];

      acceleration_constraints = [base_trajectory.kStartAcceleration_, ...
                                  base_trajectory.kFinalAcceleration_];

      base_trajectory.position_.plan(time_constraints, position_constraints, velocity_constraints, acceleration_constraints);

      base_trajectory.position_.storePlannedTrajectory(base_trajectory.kStartTime_, motion_duration);
    end

    function update(base_trajectory, current_time, motion_start_time, motion_final_time)
      arguments (Input)
        base_trajectory;
        current_time      (1, 1) {mustBeA(current_time,      "double")};
        motion_start_time (1, 1) {mustBeA(motion_start_time, "double")};
        motion_final_time (1, 1) {mustBeA(motion_final_time, "double")};
      end

      base_trajectory.position_.update(current_time, motion_start_time, motion_final_time);
    end

  end

end  % BaseTrajectory
