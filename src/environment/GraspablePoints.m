classdef GraspablePoints < handle
% Graspable points parameters
%
% Created     : 2020.05.12 by Yusuke Koizumi
% Last updated: 2025.01.06 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    kDetectionType_ (1, 1) string;
    point_cloud_    (3, :) double;  % Graspable points position described in world frame [m]
  end
  properties (SetAccess = private, GetAccess = private)
    graphics_ (1, 1) matlab.graphics.chart.primitive.Scatter;
    kVisualization_ (1, 1) logical;
    kMarkerStyle_ (1, 1) string;
    kMarkerSize_ (1, 1) double;
    kColor_ (1, 3) double;  % [0, 1]
    kTransparency_ (1, 1) double;  % [0, 1]
  end

  %% Public Methods
  methods (Access = public)

    function graspable_points = GraspablePoints()
    % Constructor
      graspable_points.kDetectionType_ = strings;
    end

    function setVisualSettings(graspable_points, visibility, marker_style, marker_size, color, transparency)
      arguments (Input)
        graspable_points;
        visibility   (1, 1) {mustBeA(visibility, "logical")};
        marker_style (1, 1) {mustBeA(marker_style, "string")};
        marker_size  (1, 1) {mustBeA(marker_size, "double")};
        color        (:, 1) {mustBeA(color, ["double", "string"])};
        transparency (1, 1) {mustBeA(transparency, "double")};
      end

      graspable_points.kVisualization_ = visibility;
      graspable_points.kMarkerStyle_ = marker_style;
      graspable_points.kMarkerSize_ = marker_size;
      graspable_points.kColor_ = color;
      graspable_points.kTransparency_ = transparency;

      if (graspable_points.kVisualization_)
        graspable_points.createGraspablePointsGraphics();
      end
    end

    function visualize(graspable_points)
      if (graspable_points.kVisualization_)
        graspable_points.graphics_.XData = graspable_points.point_cloud_(1, :);
        graspable_points.graphics_.YData = graspable_points.point_cloud_(2, :);
        graspable_points.graphics_.ZData = graspable_points.point_cloud_(3, :);
        graspable_points.graphics_.Visible = "on";
      end
    end

  end

  %% Methods called only from Terrain
  methods (Access = ?Terrain)

    function setDetectionType(graspable_points, config_terrain)
      arguments (Input)
        graspable_points;
        config_terrain (1, 1) {mustBeA(config_terrain, "ConfigTerrain")};
      end

      graspable_points.kDetectionType_ = config_terrain.getGraspablePointsDetectionType();
    end

    function setGraspablePoints(graspable_points, terrain_points)
      arguments (Input)
        graspable_points;
        terrain_points (3, :) {mustBeA(terrain_points, "double")};
      end

      switch graspable_points.kDetectionType_
        case "all"
          point_cloud = terrain_points;
        otherwise
          error("ERROR: Failed to set graspable points.");
      end

      graspable_points.point_cloud_ = point_cloud;
    end

  end

  %% Methods called only from Perception
  methods (Access = ?Perception)

    function setSensedGraspablePoints(graspable_points, sensed_graspable_points)
      arguments (Input)
        graspable_points;
        sensed_graspable_points (3, :) {mustBeA(sensed_graspable_points, "double")};
      end
      graspable_points.point_cloud_ = sensed_graspable_points;
    end

  end

  %% Private Methods
  methods (Access = private)

    function createGraspablePointsGraphics(graspable_points)
      graspable_points.graphics_ = scatter3( ...
        graspable_points.point_cloud_(1, :), ...
        graspable_points.point_cloud_(2, :), ...
        graspable_points.point_cloud_(3, :), ...
        Marker = graspable_points.kMarkerStyle_, ...
        SizeData = graspable_points.kMarkerSize_, ...
        MarkerFaceColor = graspable_points.kColor_, ...
        MarkerEdgeColor = "none", ...
        MarkerFaceAlpha = graspable_points.kTransparency_, ...
        Visible = "off");
    end

  end

  %% Getter
  methods (Access = public)

    function kPointCloud = getPointCloud(graspable_points)
      kPointCloud = graspable_points.point_cloud_;
    end

    function nearest_point = getNearestPoint(graspable_points, original_position)
      arguments (Input)
        graspable_points;
        original_position (3, 1) {mustBeA(original_position, "double")};
      end

      [~, nearest_GPs_id] = min( ...
        (graspable_points.point_cloud_(1, :) - original_position(1, 1)) .^ 2 + ...
        (graspable_points.point_cloud_(2, :) - original_position(2, 1)) .^ 2 + ...
        (graspable_points.point_cloud_(3, :) - original_position(3, 1)) .^ 2);

      nearest_point = [ graspable_points.point_cloud_(1, nearest_GPs_id); ...
                        graspable_points.point_cloud_(2, nearest_GPs_id); ...
                        graspable_points.point_cloud_(3, nearest_GPs_id)];
    end

  end

end  % GraspablePoints
