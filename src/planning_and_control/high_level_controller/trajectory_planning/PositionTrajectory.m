classdef PositionTrajectory < handle
% PositionTrajectory
% Plan position trajectory and calculate desired position at current time step
%
% Created     : 2024.05.20 by Masazumi Imai
% Last updated: 2024.12.12 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    planner_;
    desired_position_   (3, 1) double;  % [m]
    planned_trajectory_ (1, 1) TrajectoryHistory;
  end

  %% Public Methods
  methods (Access = public)

    % Constructor
    function position_trajectory = PositionTrajectory(type)
      arguments (Input)
        type (1, 1) {mustBeA(type, "string")};
      end

      switch (type)
        case "5th_order_bezier"
          position_trajectory.planner_ = FifthOrderBezier();
        case "7th_order_bezier"
          position_trajectory.planner_ = SeventhOrderBezier();
        case "7th_order_spline"
          position_trajectory.planner_ = SeventhOrderSpline();
        otherwise
          error("Invalid position trajectory type is specified.");
      end

      position_trajectory.desired_position_ = [0.0; 0.0; 0.0];
      position_trajectory.planned_trajectory_ = TrajectoryHistory();
    end

    function plan(position_trajectory, ...
        time_constraints, position_constraints, velocity_constraints, acceleration_constraints)
      arguments (Input)
        position_trajectory;
        time_constraints         (1, :) {mustBeA(time_constraints, "double")};
        position_constraints     (3, :) {mustBeA(position_constraints, "double")};
        velocity_constraints     (3, :) {mustBeA(velocity_constraints, "double")};
        acceleration_constraints (3, :) {mustBeA(acceleration_constraints, "double")};
      end

      position_trajectory.planner_.calcControlPoints( ...
        time_constraints, position_constraints, velocity_constraints, acceleration_constraints);
    end

    function storePlannedTrajectory(position_trajectory, ...
        start_time, final_time)
      arguments (Input)
        position_trajectory;
        start_time (1, 1) {mustBeA(start_time, "double")};
        final_time (1, 1) {mustBeA(final_time, "double")};
      end

      kTimeStep = 0.01;
      for time = 0.0 : kTimeStep : final_time
        desired_position = position_trajectory.planner_.calcDesiredPositionForCurrentTimeStep( ...
          time, start_time, final_time);

        position_trajectory.planned_trajectory_.addPoint(desired_position);
      end
    end

    function update(position_trajectory, current_time, start_time, final_time)
      arguments (Input)
        position_trajectory;
        current_time (1, 1) {mustBeA(current_time, "double")};
        start_time   (1, 1) {mustBeA(start_time, "double")};
        final_time   (1, 1) {mustBeA(final_time, "double")};
      end

      position_trajectory.desired_position_ = ...
        position_trajectory.planner_.calcDesiredPositionForCurrentTimeStep( ...
          current_time, start_time, final_time);
    end

    function stay(position_trajectory, current_EE_position)
      position_trajectory.desired_position_ = current_EE_position;
    end

  end

  %% Getter
  methods (Access = public)
    function desired_position = getDesiredPosition(position_trajectory)
      desired_position = position_trajectory.desired_position_;
    end
  end

end  % PositionTrajectory
