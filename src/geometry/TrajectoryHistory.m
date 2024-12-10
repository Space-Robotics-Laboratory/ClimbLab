classdef TrajectoryHistory < handle
% TrajectoryHistory
%
% Created     : 2024.05.20 by Masazumi Imai
% Last updated: 2024.12.07 by Masazumi Imai

%% Properties
  properties (SetAccess = private, GetAccess = public)
    points_ (3, :) double;
  end
  properties (SetAccess = private, GetAccess = private)
    kLineStyle_ (1, 1) string;
    kColor_      (1, 3) double;
    kWidth_      (1, 1) double;
    line_       (1, 1) matlab.graphics.animation.AnimatedLine;
  end

  %% Public Methods
  methods (Access = public)

    function trajectory = TrajectoryHistory()
    % TrajectoryHistory() Constructor
      trajectory.points_ = double.empty;

      trajectory.kLineStyle_ = "none";
      trajectory.kColor_ = [0.0, 0.0, 0.0];
      trajectory.kWidth_ = 0.0;
    end

    function addPoint(trajectory, point)
      arguments (Input)
        trajectory;
        point (3, 1) {mustBeA(point, "double")};
      end

      trajectory.points_ = horzcat(trajectory.points_, point);

      if (trajectory.kLineStyle_ == "none")
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

    function setVisualSettings(trajectory, line_style, color, width)
      arguments (Input)
        trajectory;
        line_style (1, 1) {mustBeA(line_style, "string")} = "none";
        color = [0.0, 0.0, 0.0];
        width      (1, 1) {mustBeA(width,      "double")} = 0.0;
      end

      trajectory.kLineStyle_ = line_style;
      trajectory.kColor_ = validatecolor(color);
      trajectory.kWidth_ = width;

      if (line_style == "none")
        return;
      end

      trajectory.line_ = animatedline( ...
        LineStyle = trajectory.kLineStyle_, ...
        Color = trajectory.kColor_, ...
        LineWidth = trajectory.kWidth_, ...
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
