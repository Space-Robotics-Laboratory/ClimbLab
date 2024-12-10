classdef GraspablePoints < handle

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    kDetectionType_ (1, 1) string;
    kPointCloud_    (3, :) double;
  end
  properties (SetAccess = private, GetAccess = private)
    kMarkerStyle_ (1, 1) string;
    kMarkerSize_ (1, 1) double;
    kColor_ (1, 3) double;  % [0, 1]
    kTransparency_ (1, 1) double;  % [0, 1]
  end

  %% Methods called only from Terrain
  methods (Access = ?Terrain)

    % Constructor
    function graspable_points = GraspablePoints(config_terrain)
      arguments (Input)
        config_terrain (1, 1) {mustBeA(config_terrain, "ConfigTerrain")};
      end
      graspable_points.kDetectionType_ = config_terrain.getGraspablePointsDetectionType();

      [graspable_points.kMarkerStyle_, graspable_points.kMarkerSize_, graspable_points.kColor_, ...
        graspable_points.kTransparency_] = config_terrain.getGraspablePointsVisualSettings();
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

      graspable_points.kPointCloud_ = point_cloud;
    end

  end

  %% Public Methods
  methods (Access = public)
    function visualize(graspable_points)
      scatter3( ...
        graspable_points.kPointCloud_(1, :), ...
        graspable_points.kPointCloud_(2, :), ...
        graspable_points.kPointCloud_(3, :), ...
        Marker = graspable_points.kMarkerStyle_, ...
        SizeData = graspable_points.kMarkerSize_, ...
        MarkerFaceColor = validatecolor(graspable_points.kColor_), ...
        MarkerEdgeColor = "none", ...
        MarkerFaceAlpha = graspable_points.kTransparency_);
    end
  end

  %% Private Methods
  methods (Access = private)
  end

  %% Getter
  methods (Access = public)
    function nearest_point = getNearestPoint(graspable_points, original_position)
      arguments (Input)
        graspable_points;
        original_position (3, 1) {mustBeA(original_position, "double")};
      end

      [~, nearest_GPs_id] = min( ...
        (graspable_points.kPointCloud_(1, :) - original_position(1, 1)) .^ 2 + ...
        (graspable_points.kPointCloud_(2, :) - original_position(2, 1)) .^ 2 + ...
        (graspable_points.kPointCloud_(3, :) - original_position(3, 1)) .^ 2);

      nearest_point = [ graspable_points.kPointCloud_(1, nearest_GPs_id); ...
                        graspable_points.kPointCloud_(2, nearest_GPs_id); ...
                        graspable_points.kPointCloud_(3, nearest_GPs_id)];
    end
  end

end  % GraspablePoints
