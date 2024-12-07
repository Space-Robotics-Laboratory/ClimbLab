classdef BaseTrajectory
% BaseTrajectory
% Plan robot base pose trajectory and calculate desired pose at current time step
%
% Created     : 2024.05.20 by Masazumi Imai
% Last updated: 2024.12.07 by Masazumi Imai

  %% Properties
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

  %% Methods called only from MotionPlanning
  methods (Access = ?MotionPlanning)

    % Constructor
    function base_trajectory = BaseTrajectory(config)
      arguments (Input)
        config (1, 1) {mustBeA(config, "ConfigMotionPlanning")};
      end
      [type, ~] = config.getTrajectoryType();

      base_trajectory.position_ = PositionTrajectory(type);


      line_style = "none"; color = [0.0, 0.0, 0.0]; width = 0.0;
      base_trajectory.position_ = base_trajectory.position_.setVisualSettings( ...
        line_style, color, width);
    end

    function base_trajectory = plan(base_trajectory, robot, gait_planning)
    % plan()
    %   Plan the trajectory from current to desired pose of the robot base
      arguments (Input)
        base_trajectory;
        robot         (1, 1) {mustBeA(robot, "Robot")};
        gait_planning (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
      end

      current_base_position = robot.SV.getBasePosition();  % previous desired position?
      desired_base_position = gait_planning.base_pose_planner_.getDesiredBasePosition();
      motion_duration = gait_planning.getTransferDuration();

      time_constraints = [base_trajectory.kStartTime_, motion_duration];
      position_constraints = [current_base_position, desired_base_position];
      velocity_constraints = [base_trajectory.kStartVelocity_, base_trajectory.kFinalVelocity_];
      acceleration_constraints = [base_trajectory.kStartAcceleration_, ...
                                  base_trajectory.kFinalAcceleration_];

      base_trajectory.position_ = base_trajectory.position_.plan( ...
        time_constraints, position_constraints, velocity_constraints, acceleration_constraints);

      base_trajectory.position_ = base_trajectory.position_.storePlannedTrajectory( ...
        base_trajectory.kStartTime_, motion_duration);
    end

    function base_trajectory = update(base_trajectory, current_time, motion_start_time, motion_final_time)
      arguments (Input)
        base_trajectory;
        current_time    (1, 1) {mustBeA(current_time, "double")};
        motion_start_time (1, 1) {mustBeA(motion_start_time, "double")};
        motion_final_time (1, 1) {mustBeA(motion_final_time, "double")};
      end

      base_trajectory.position_ = base_trajectory.position_.update( ...
        current_time, motion_start_time, motion_final_time);
    end

  end

end  % BaseTrajectory
