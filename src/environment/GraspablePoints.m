classdef GraspablePoints

  properties (SetAccess = private, GetAccess = public)
    detection_type (1, 1) string;
    point_cloud (3, :) double;
  end

  methods (Access = ?MapSurface)
    % Constructor
    function graspable_points = GraspablePoints()
      graspable_points.detection_type = "";
    end

    function graspable_points = initialize(graspable_points, detection_type, map_surface_points)
      arguments (Input)
        graspable_points;
        detection_type     (1, 1) {mustBeA(detection_type,     "string")};
        map_surface_points (3, :) {mustBeA(map_surface_points, "double")};
      end
      graspable_points.detection_type = detection_type;
      graspable_points.point_cloud = graspable_points.setGraspablePoints(map_surface_points);
    end
  end  % methods (Access = ?MapSurface)

  methods (Access = private)
    function point_cloud = setGraspablePoints(graspable_points, map_surface_points)
      switch graspable_points.detection_type
        case "all"
          point_cloud = map_surface_points;
        otherwise
      end
    end
  end  % methods (Access = private)

  methods (Access = public)
    function visualize(graspable_points, marker_style, marker_size, color, alpha)
      arguments (Input)
        graspable_points;
        marker_style (1, 1) {mustBeA(marker_style, "string")};
        marker_size  (1, 1) {mustBeA(marker_size,  "double")};
        color;
        alpha        (1, 1) {mustBeA(alpha,        "double")};
      end
      scatter3( ...
        graspable_points.point_cloud(1, :), ...
        graspable_points.point_cloud(2, :), ...
        graspable_points.point_cloud(3, :), ...
        Marker = marker_style, SizeData = marker_size, ...
        MarkerFaceColor = validatecolor(color), MarkerEdgeColor = "none", MarkerFaceAlpha = alpha);
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