classdef Terrain
  %% Properties
  properties (SetAccess = private, GetAccess = public)
    type        (1, 1) string;
    inclination (3, 1) double;  % [deg]

    raw_map_data (1, 1) struct;  % (x, y, z)
    point_cloud_in_World (3, :) double;
    norm_vectors;

    stiffness_coefficient_for_GRF (1, 1) double;
    damping_coefficient_for_GRF   (1, 1) double;
    stiffness_coefficient_for_GRM (1, 1) double;
    damping_coefficient_for_GRM   (1, 1) double;

    graspable_points GraspablePoints;
  end
  properties (Access = private)
    grid_color      (1, 3) double;
    alpha           (1, 1) double;
    graphical_model (1, 1) matlab.graphics.chart.primitive.Surface;
  end

  %% Public Methods
  methods (Access = public)

    % Constructor
    function terrain = Terrain(config)
      arguments (Input)
        config (1, 1) {mustBeA(config, "ConfigTerrain")};
      end
      terrain.type = config.getSurfaceType();
      terrain.inclination = config.getSurfaceInclination();

      terrain.raw_map_data = terrain.loadSurfaceDataFromMatFile();
      terrain.point_cloud_in_World = terrain.setPointCloud();
      terrain.norm_vectors = terrain.setNormVectors();

      [terrain.stiffness_coefficient_for_GRF, terrain.damping_coefficient_for_GRF, ...
        terrain.stiffness_coefficient_for_GRM, terrain.damping_coefficient_for_GRM] = ...
        config.getGroundCoefficients();

      terrain.graspable_points = GraspablePoints(config);

      [terrain.grid_color, terrain.alpha] = config.getTerrainVisualSettings();
    end

    function terrain = initialize(terrain)
      terrain.graspable_points = ...
        terrain.graspable_points.initialize(terrain.point_cloud_in_World);

      terrain = terrain.createSurfaceGraphics();
    end

    function terrain = visualize(terrain, time)
      if (time ~= 0.0)
        return;
      end
      terrain.graphical_model.Visible = "on";
    end

  end

  %% Private Methods
  methods (Access = private)

    function raw_map_data = loadSurfaceDataFromMatFile(terrain)
      folder = "src/environment/map";
      map_file_name = "map_" + terrain.type + ".mat";
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

    function point_cloud_in_World = setPointCloud(terrain)
      x = terrain.raw_map_data.x;
      y = terrain.raw_map_data.y;
      z = terrain.raw_map_data.z;
      point_cloud_in_Surface = [repelem(x, size(y, 2)); ...
                                repmat(y, 1, size(x, 2)); ...
                                reshape(z, 1, [])];
      % inclined_surface_point_cloud = ...
      %   eul2rotm(deg2rad(terrain.inclination)', "ZYX") * point_cloud_tmp;
      inclined_surface_point_cloud = rpy2dc(deg2rad(terrain.inclination))' * ...
        point_cloud_in_Surface;
      point_cloud_in_World = [inclined_surface_point_cloud(1, :); ...
                              inclined_surface_point_cloud(2, :); ...
                              inclined_surface_point_cloud(3, :)];
    end

    function norm_vector = setNormVectors(terrain)
      [Nx, Ny, Nz] = surfnorm(terrain.raw_map_data.z);
      norm_vector_in_Surface = [reshape(Nx, 1, []); reshape(Ny, 1, []); reshape(Nz, 1, [])];
      norm_vector = rpy2dc(deg2rad(terrain.inclination))' * norm_vector_in_Surface;
    end

    function terrain = createSurfaceGraphics(terrain)
      [X, Y] = meshgrid(terrain.raw_map_data.x, terrain.raw_map_data.y);
      Z = terrain.raw_map_data.z;
      map_vec(1, :) = reshape(X, 1, numel(X));
      map_vec(2, :) = reshape(Y, 1, numel(Y));
      map_vec(3, :) = reshape(Z, 1, numel(Z));

      % inclined_map_vec = eul2rotm(deg2rad(terrain.inclination)', "ZYX") * map_vec;
      inclined_map_vec = rpy2dc(deg2rad(terrain.inclination))' * map_vec;
      inclined_X = reshape(inclined_map_vec(1, :), size(X, 1), size(X, 2));
      inclined_Y = reshape(inclined_map_vec(2, :), size(Y, 1), size(Y, 2));
      inclined_Z = reshape(inclined_map_vec(3, :), size(Z, 1), size(Z, 2));

      terrain.graphical_model = mesh(inclined_X, inclined_Y, inclined_Z, ...
        EdgeColor = terrain.grid_color, ...
        EdgeAlpha = terrain.alpha, ...
        Visible = "off");
    end

  end

  %% Getter
  methods (Access = public)
    function inclination = getSurfaceInclination(terrain)
      inclination = terrain.inclination;
    end
    function [Kf, Df, Km, Dm] = getGroundCoefficients(terrain)
      Kf = terrain.stiffness_coefficient_for_GRF;
      Df = terrain.damping_coefficient_for_GRF;
      Km = terrain.stiffness_coefficient_for_GRM;
      Dm = terrain.damping_coefficient_for_GRM;
    end
    function graspable_points = getGraspablePoints(terrain)
      graspable_points = terrain.graspable_points;
    end

    function nearest_point = getNearestPointInWorldFrame(terrain, original_point)
    % getNearestPointInWorldFrame() Obtain the correspondent map positions
    %   Obtain the closest point in the map for a point
    %   HACK: Don't use point cloud to obtain the closest point for high computation speed
    %   Created:      2019.09.30 by Victoria Keo, Warley Ribeiro
    %   Last updated: 2024.03.15 by Masazumi Imai
    %
    % Input : original_point (3x1) - Given position of the point to be checked in the World frame
    % Output: nearest_point (3x1) - Closest point positions for the map in the World frame
      arguments (Input)
        terrain;
        original_point (3, 1) {mustBeA(original_point, "double")};
      end

      original_point_in_Surface = rpy2dc(deg2rad(terrain.inclination)) * original_point;

      [~, id_x] = min(abs(terrain.raw_map_data.x - original_point_in_Surface(1, 1)));
      nearest_point_in_Surface(1, 1) = terrain.raw_map_data.x(1, id_x);

      [~, id_y] = min(abs(terrain.raw_map_data.y - original_point_in_Surface(2, 1)));
      nearest_point_in_Surface(2, 1) = terrain.raw_map_data.y(1, id_y);

      nearest_point_in_Surface(3, 1) = terrain.raw_map_data.z(id_y, id_x);

      nearest_point = rpy2dc(deg2rad(terrain.inclination))' * nearest_point_in_Surface;
    end

    function norm_vector_at_point = getNormVectorAtPoint(terrain, point)
      arguments (Input)
        terrain;
        point (3, 1) {mustBeA(point, "double")};
      end

      [~, idx] = min(vecnorm(terrain.point_cloud_in_World - point));
      norm_vector_at_point = terrain.norm_vectors(:, idx);
    end
  end

end
% EOF
