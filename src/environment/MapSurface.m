classdef MapSurface

  properties (SetAccess = private, GetAccess = public)
    type (1, 1) string;
    inclination (3, 1) double;

    raw_map_data (1, 1) struct;  % (x, y, z)
    point_cloud  (3, :) double;

    graspable_points GraspablePoints;
  end
  properties (Access = private)
    grid_color      (1, 3) double;
    alpha           (1, 1) double;
    graphical_model (1, 1) matlab.graphics.chart.primitive.Surface;
  end

  methods (Access = public)
    % Constructor
    function map_surface = MapSurface(map_surface_type, surface_inclination)
      arguments (Input)
        map_surface_type    (1, 1) {mustBeA(map_surface_type,    "string")};
        surface_inclination (3, 1) {mustBeA(surface_inclination, "double")};
      end
      map_surface.type = map_surface_type;
      map_surface.inclination = surface_inclination;

      map_surface.raw_map_data = map_surface.loadSurfaceDataFromMatFile();
      map_surface.point_cloud = map_surface.getPointCloud();

      map_surface.graspable_points = GraspablePoints();

      map_surface.grid_color = [0.0, 0.0, 0.0];
      map_surface.alpha = 0.0;
    end

    function map_surface = visualize(map_surface, surface_grid_color, surface_alpha)
      arguments (Input)
        map_surface;
        surface_grid_color;
        surface_alpha (1, 1) {mustBeA(surface_alpha, "double")};
      end
      [X, Y] = meshgrid(map_surface.raw_map_data.x, map_surface.raw_map_data.y);
      Z = map_surface.raw_map_data.z;
      map_vec(1, :) = reshape(X, 1, numel(X));
      map_vec(2, :) = reshape(Y, 1, numel(Y));
      map_vec(3, :) = reshape(Z, 1, numel(Z));

      inclined_map_vec = eul2rotm(deg2rad(map_surface.inclination)', "ZYX") * map_vec;
      inclined_X = reshape(inclined_map_vec(1, :), size(X, 1), size(X, 2));
      inclined_Y = reshape(inclined_map_vec(2, :), size(Y, 1), size(Y, 2));
      inclined_Z = reshape(inclined_map_vec(3, :), size(Z, 1), size(Z, 2));

      map_surface.graphical_model = mesh(inclined_X, inclined_Y, inclined_Z, ...
        EdgeColor = surface_grid_color, ...
        EdgeAlpha = surface_alpha);
    end

    function map_surface = initializeGraspablePoints(map_surface, graspable_points_detection_type)
      arguments (Input)
        map_surface;
        graspable_points_detection_type ...
          (1, 1) {mustBeA(graspable_points_detection_type, "string")};
      end

      map_surface.graspable_points = map_surface.graspable_points.initialize( ...
        graspable_points_detection_type, map_surface.point_cloud);
    end

    function graspable_points = getGraspablePoints(map_surface)
      graspable_points = map_surface.graspable_points;
    end
  end

  methods (Access = private)
    function raw_map_data = loadSurfaceDataFromMatFile(map_surface)
      folder = "src/environment/map";
      map_file_name = "map_" + map_surface.type + ".mat";
      file_path = fullfile(folder, map_file_name);
      if ~exist(file_path, "file")
        error("Invalid surface type is specified" + newline ...
          + "Check ""type"" defined in config file.");
      end
      load(file_path, "x", "y", "z");
      raw_map_data.x = x;
      raw_map_data.y = y;
      raw_map_data.z = z;
    end

    function point_cloud = getPointCloud(map_surface)
      x = map_surface.raw_map_data.x;
      y = map_surface.raw_map_data.y;
      z = map_surface.raw_map_data.z;
      point_cloud_tmp = [ repelem(x, size(y, 2)); ...
                          repmat(y, 1, size(x, 2)); ...
                          reshape(z, 1, [])];
      inclined_surface_point_cloud = ...
        eul2rotm(deg2rad(map_surface.inclination)', "ZYX") * point_cloud_tmp;
      point_cloud = [ inclined_surface_point_cloud(1, :); ...
                      inclined_surface_point_cloud(2, :); ...
                      inclined_surface_point_cloud(3, :)];
    end
  end

end
% EOF