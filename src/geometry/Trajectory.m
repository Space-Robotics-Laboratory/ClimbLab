classdef Trajectory
  %% Properties
  properties (SetAccess = private, GetAccess = public)
    points (3, :) double;
  end
  properties (SetAccess = private, GetAccess = private)
    line_style (1, 1) string;
    color      (1, 3) double;
    width      (1, 1) double;
    line       (1, 1) matlab.graphics.animation.AnimatedLine;
  end

  %% Public methods
  methods (Access = public)

    function trajectory = Trajectory()
    % Trajectory() Constructor
      trajectory.points = double.empty;

      trajectory.line_style = "none";
      trajectory.color = [0.0, 0.0, 0.0];
      trajectory.width = 0.0;
    end

    function trajectory = addPoint(trajectory, point)
      arguments (Input)
        trajectory;
        point (3, 1) {mustBeA(point, "double")};
      end

      trajectory.points = horzcat(trajectory.points, point);

      if (trajectory.line_style == "none")
        return;
      end
      addpoints(trajectory.line, point(1, 1), point(2, 1), point(3, 1));
    end

    function visualize(trajectory)
      trajectory.line.Visible = "on";
    end

  end

  %% Setter
  methods (Access = public)
    function trajectory = setVisualSettings(trajectory, line_style, color, width)
      arguments (Input)
        trajectory;
        line_style (1, 1) {mustBeA(line_style, "string")} = "none";
        color = [0.0, 0.0, 0.0];
        width      (1, 1) {mustBeA(width,      "double")} = 0.0;
      end
      trajectory.line_style = line_style;
      trajectory.color = validatecolor(color);
      trajectory.width = width;

      if (line_style == "none")
        return;
      end

      trajectory.line = animatedline( ...
        LineStyle = trajectory.line_style, ...
        Color = trajectory.color, ...
        LineWidth = trajectory.width, ...
        MaximumNumPoints = Inf, ...
        Visible = "off");
    end
  end

  %% Getter
  methods (Access = public)
    function points = getPoints(trajectory)
      points = trajectory.points;
    end
  end

end
% EOF