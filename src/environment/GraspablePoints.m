classdef GraspablePoints

  properties (SetAccess = private, GetAccess = public)
    detection_type (1, 1) string;
    point_cloud    (3, :) double;
  end
  properties (SetAccess = private, GetAccess = private)
    marker_style (1, 1) string;
    marker_size (1, 1) double;
    color (1, 3) double;  % [0, 1]
    alpha (1, 1) double;  % [0, 1]
  end

  methods (Access = ?Terrain)
    % Constructor
    function graspable_points = GraspablePoints(config)
      arguments (Input)
        config (1, 1) {mustBeA(config, "ConfigTerrain")};
      end
      graspable_points.detection_type = config.getGraspablePointsDetectionType();

      [graspable_points.marker_style, graspable_points.marker_size, graspable_points.color, ...
        graspable_points.alpha] = config.getGraspablePointsVisualSettings();
    end

    function graspable_points = initialize(graspable_points, terrain_points)
      arguments (Input)
        graspable_points;
        terrain_points (3, :) {mustBeA(terrain_points, "double")};
      end
      graspable_points.point_cloud = graspable_points.setGraspablePoints(terrain_points);
    end
  end  % methods (Access = ?Terrain)

  methods (Access = private)
    function point_cloud = setGraspablePoints(graspable_points, terrain_points)
      switch graspable_points.detection_type
        case "all"
          point_cloud = terrain_points;
        otherwise
      end
    end
  end  % methods (Access = private)

  methods (Access = public)
    function visualize(graspable_points)
      scatter3( ...
        graspable_points.point_cloud(1, :), ...
        graspable_points.point_cloud(2, :), ...
        graspable_points.point_cloud(3, :), ...
        Marker = graspable_points.marker_style, SizeData = graspable_points.marker_size, ...
        MarkerFaceColor = validatecolor(graspable_points.color), ...
        MarkerEdgeColor = "none", MarkerFaceAlpha = graspable_points.alpha);
    end

    function nearest_point = getNearestPoint(graspable_points, original_position)
      arguments (Input)
        graspable_points;
        original_position (3, 1) {mustBeA(original_position, "double")};
      end
      [~, nearest_GPs_id] = min( ...
        (graspable_points.point_cloud(1, :) - original_position(1, 1)).^2 + ...
        (graspable_points.point_cloud(2, :) - original_position(2, 1)).^2 + ...
        (graspable_points.point_cloud(3, :) - original_position(3, 1)).^2);

      nearest_point = [graspable_points.point_cloud(1, nearest_GPs_id); ...
        graspable_points.point_cloud(2, nearest_GPs_id); ...
        graspable_points.point_cloud(3, nearest_GPs_id)];
    end
  end  % methods (Access = public)

end
% EOF