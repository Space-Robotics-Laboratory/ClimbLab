classdef TrajectoryHistory
% TrajectoryHistory
%
% Created     : 2024.05.20 by Masazumi Imai
% Last updated: 2024.12.07 by Masazumi Imai

%% Properties
  properties (SetAccess = private, GetAccess = public)
    points_ (3, :) double;
  end
  properties (SetAccess = private, GetAccess = private)
    line_style_ (1, 1) string;
    color_      (1, 3) double;
    width_      (1, 1) double;
    line_       (1, 1) matlab.graphics.animation.AnimatedLine;
  end

  %% Public Methods
  methods (Access = public)

    function trajectory = TrajectoryHistory()
    % TrajectoryHistory() Constructor
      trajectory.points_ = double.empty;

      trajectory.line_style_ = "none";
      trajectory.color_ = [0.0, 0.0, 0.0];
      trajectory.width_ = 0.0;
    end

    function trajectory = addPoint(trajectory, point)
      arguments (Input)
        trajectory;
        point (3, 1) {mustBeA(point, "double")};
      end

      trajectory.points_ = horzcat(trajectory.points_, point);

      if (trajectory.line_style_ == "none")
        return;
      end
      addpoints(trajectory.line_, point(1, 1), point(2, 1), point(3, 1));
    end

    function visualize(trajectory)
      trajectory.line_.Visible = "on";
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

      trajectory.line_style_ = line_style;
      trajectory.color_ = validatecolor(color);
      trajectory.width_ = width;

      if (line_style == "none")
        return;
      end

      trajectory.line_ = animatedline( ...
        LineStyle = trajectory.line_style_, ...
        Color = trajectory.color_, ...
        LineWidth = trajectory.width_, ...
        MaximumNumPoints = Inf, ...
        Visible = "off");
    end
  end

  %% Getter
  methods (Access = public)
    function points = getPoints(trajectory)
      points = trajectory.points_;
    end
  end

end  % TrajectoryHistory
