classdef Trajectory

  properties (SetAccess = private, GetAccess = public)
    points (3, :) double;
  end
  properties (SetAccess = private, GetAccess = private)
    line_style (1, 1) string;
    color      (1, 3) double;
    width      (1, 1) double;
    line       (1, 1) matlab.graphics.animation.AnimatedLine;
  end

  methods (Access = public)
    % Constructor
    function trajectory = Trajectory()
      trajectory.points = double.empty;

      trajectory.line_style = "none";
      trajectory.color = [0.0, 0.0, 0.0];
      trajectory.width = 0.0;
    end

    function trajectory = initialize(trajectory, point, line_style, color, width)
      arguments (Input)
        trajectory;
        point      (3, 1) {mustBeA(point,      "double")};
        line_style (1, 1) {mustBeA(line_style, "string")} = "none";
        color = [0.0, 0.0, 0.0];
        width      (1, 1) {mustBeA(width,      "double")} = 0.0;
      end
      trajectory.points = point;

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

end
% EOF