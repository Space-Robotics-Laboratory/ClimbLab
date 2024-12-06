classdef PositionTrajectory
  %% Properties
  properties (SetAccess = private, GetAccess = public)
    planner;
    desired_position   (3, 1) double;  % [m]
    planned_trajectory (1, 1) Trajectory;
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
          position_trajectory.planner = FifthOrderBezier();
        case "7th_order_bezier"
          position_trajectory.planner = SeventhOrderBezier();
        case "7th_order_spline"
          position_trajectory.planner = SeventhOrderSpline();
        otherwise
          error("Invalid position trajectory type is specified!!");
      end

      position_trajectory.desired_position = [0.0; 0.0; 0.0];
      position_trajectory.planned_trajectory = Trajectory();
    end

    function position_trajectory = plan(position_trajectory, ...
        time_constraints, position_constraints, velocity_constraints, acceleration_constraints)
      arguments (Input)
        position_trajectory;
        time_constraints         (1, :) {mustBeA(time_constraints, "double")};
        position_constraints     (3, :) {mustBeA(position_constraints, "double")};
        velocity_constraints     (3, :) {mustBeA(velocity_constraints, "double")};
        acceleration_constraints (3, :) {mustBeA(acceleration_constraints, "double")};
      end

      position_trajectory.planner = position_trajectory.planner.calcCoefficients( ...
        time_constraints, position_constraints, velocity_constraints, acceleration_constraints);
    end

    function position_trajectory = storePlannedTrajectory(position_trajectory, ...
        start_time, final_time)
      arguments (Input)
        position_trajectory;
        start_time (1, 1) {mustBeA(start_time, "double")};
        final_time (1, 1) {mustBeA(final_time, "double")};
      end

      time_step = 0.01;
      for time = 0.0 : time_step : final_time
        desired_position_ = position_trajectory.planner.calcDesiredPositionForCurrentTimeStep( ...
          time, start_time, final_time);

        position_trajectory.planned_trajectory = ...
          position_trajectory.planned_trajectory.addPoint(desired_position_);
      end
    end

    function position_trajectory = update(position_trajectory, current_time, start_time, final_time)
      arguments (Input)
        position_trajectory;
        current_time (1, 1) {mustBeA(current_time, "double")};
        start_time   (1, 1) {mustBeA(start_time, "double")};
        final_time   (1, 1) {mustBeA(final_time, "double")};
      end

      position_trajectory.desired_position = ...
        position_trajectory.planner.calcDesiredPositionForCurrentTimeStep( ...
          current_time, start_time, final_time);
    end

    function position_trajectory = stay(position_trajectory, current_EE_position)
      position_trajectory.desired_position = current_EE_position;
    end

    function position_trajectory = setVisualSettings(position_trajectory, line_style, color, width)
      arguments (Input)
        position_trajectory;
        line_style (1, 1) {mustBeA(line_style, "string")};
        color;
        width      (1, 1) {mustBeA(width,      "double")};
      end

      position_trajectory.planned_trajectory = ...
        position_trajectory.planned_trajectory.setVisualSettings(line_style, color, width);
    end

  end

  %% Getter
  methods (Access = public)
    function desired_position = getDesiredPosition(position_trajectory)
      desired_position = position_trajectory.desired_position;
    end
  end

end
% EOF