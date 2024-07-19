classdef EEPositionTrajectory

  properties (SetAccess = private, GetAccess = public)
    planner;
    desired_EE_position (3, 1) double;
    planned_trajectory  (1, 1) Trajectory;
  end
  properties (Constant, GetAccess = private)
    start_time         (1, 1) double = 0.0;
    start_velocity     (3, 1) double = [0.0; 0.0; 0.0];
    final_velocity     (3, 1) double = [0.0; 0.0; 0.0];
    start_acceleration (3, 1) double = [0.0; 0.0; 0.0];
    final_acceleration (3, 1) double = [0.0; 0.0; 0.0];
  end
  properties (SetAccess = private, GetAccess = public)
    mid_time     (1, 1) double;
    mid_position (3, 1) double;
    mid_velocity (3, 1) double;
  end

  methods (Access = ?LimbEETrajectory)
    % Constructor
    function EE_position_trajectory = EEPositionTrajectory(trajectory_type)
      switch trajectory_type
        case "7th_order_bezier"
          EE_position_trajectory.planner = SeventhOrderBezier();
        case "7th_order_spline"
          EE_position_trajectory.planner = SeventhOrderSpline();
        otherwise
          error("Invalid position trajectory type for limb end-effector is specified!!");
      end

      EE_position_trajectory.desired_EE_position = [0.0; 0.0; 0.0];
      EE_position_trajectory.planned_trajectory = Trajectory();

      EE_position_trajectory.mid_time = 0.0;
      EE_position_trajectory.mid_position = [0.0; 0.0; 0.0];
      EE_position_trajectory.mid_velocity = [0.0; 0.0; 0.0];
    end

    function EE_position_trajectory = initialize(EE_position_trajectory, ...
        current_EE_position, line_style, color, width)
      EE_position_trajectory.planned_trajectory = ...
        EE_position_trajectory.planned_trajectory.initialize( ...
          current_EE_position, line_style, color, width);
    end

    function EE_position_trajectory = plan(EE_position_trajectory, ...
        final_time, start_position, final_position, step_height)
      arguments (Input)
        EE_position_trajectory;
        final_time     (1, 1) {mustBeA(final_time,     "double")};
        start_position (3, 1) {mustBeA(start_position, "double")};
        final_position (3, 1) {mustBeA(final_position, "double")};
        step_height    (1, 1) {mustBeA(step_height,    "double")};
      end

      EE_position_trajectory = EE_position_trajectory.calcMidTime(final_time);
      EE_position_trajectory = EE_position_trajectory.calcMidPosition( ...
        start_position, final_position, step_height);
      EE_position_trajectory = EE_position_trajectory.calcMidVelocity( ...
        final_time, start_position, final_position);

      start_time_ = EE_position_trajectory.start_time;
      mid_time_ = EE_position_trajectory.mid_time;
      mid_position_ = EE_position_trajectory.mid_position;
      start_velocity_ = EE_position_trajectory.start_velocity;
      mid_velocity_ = EE_position_trajectory.mid_velocity;
      final_velocity_ = EE_position_trajectory.final_velocity;
      start_acceleration_ = EE_position_trajectory.start_acceleration;
      final_acceleration_ = EE_position_trajectory.final_acceleration;

      EE_position_trajectory.planner = EE_position_trajectory.planner.calcCoefficients( ...
        start_time_, mid_time_, final_time, ...
        start_position, mid_position_, final_position, ...
        start_velocity_, mid_velocity_, final_velocity_, ...
        start_acceleration_, final_acceleration_);
    end

    function EE_position_trajectory = storePlannedTrajectory(EE_position_trajectory, ...
        final_time)
      time_step = 0.01;
      start_time_ = EE_position_trajectory.start_time;
      for time = 0.0:time_step:final_time
        desired_EE_position_ = ...
          EE_position_trajectory.planner.calcDesiredPositionForCurrentTimeStep( ...
            time, start_time_, final_time);

        EE_position_trajectory.planned_trajectory = ...
          EE_position_trajectory.planned_trajectory.addPoint(desired_EE_position_);
      end
    end

    function EE_position_trajectory = update(EE_position_trajectory, ...
        current_time, final_time)
      arguments (Input)
        EE_position_trajectory;
        current_time (1, 1) {mustBeA(current_time, "double")};
        final_time   (1, 1) {mustBeA(final_time,   "double")};
      end
      start_time_ = EE_position_trajectory.start_time;

      desired_EE_position_ = ...
        EE_position_trajectory.planner.calcDesiredPositionForCurrentTimeStep( ...
          current_time, start_time_, final_time);

      EE_position_trajectory.desired_EE_position = desired_EE_position_;
    end
  end  % methods (Access = ?LimbEETrajectory)

  methods (Access = private)
    function EE_position_trajectory = calcMidTime(EE_position_trajectory, final_time)
      EE_position_trajectory.mid_time = (EE_position_trajectory.start_time + final_time) / 2;
    end

    function EE_position_trajectory = calcMidPosition(EE_position_trajectory, ...
        start_position, final_position, step_height)
      % TODO: Need to update
      EE_position_trajectory.mid_position = ...
        (start_position + final_position) / 2 + [0.0; 0.0; step_height];
    end

    function EE_position_trajectory = calcMidVelocity(EE_position_trajectory, ...
      final_time, start_position, final_position)
      EE_position_trajectory.mid_velocity = ...
        2 * (final_position - start_position) / (final_time - EE_position_trajectory.start_time);
    end
  end  % methods (Access = private)

end
% EOF