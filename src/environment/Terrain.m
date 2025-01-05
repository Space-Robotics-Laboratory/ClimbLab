classdef Terrain < handle
% Terrain (map surface) parameters
%
% Created     : 2020.04.06 by Warley Ribeiro
% Last updated: 2025.01.05 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    kType_        (1, 1) string;
    kInclination_ (3, 1) double;  % [deg]

    kRawMapData_ (1, 1) struct;  % (x, y, z)
    kPointCloudInWorld_ (3, :) double;
    kNormalVectors_ (3, :) double;

    kStiffnessCoefficientForGRF_ (1, 1) double;
    kDampingCoefficientForGRF_   (1, 1) double;
    kStiffnessCoefficientForGRM_ (1, 1) double;
    kDampingCoefficientForGRM_   (1, 1) double;

    graspable_points_ GraspablePoints;
  end
  properties (Access = private)
    kPointDx_ (1, 1) double;

    kGridColor_    (1, 3) double;
    kTransparency_ (1, 1) double;
    graphics_ (1, 1) matlab.graphics.chart.primitive.Surface;
  end

  %% Public Methods
  methods (Access = public)

    function terrain = Terrain(config_terrain)
    % Constructor
      arguments (Input)
        config_terrain (1, 1) {mustBeA(config_terrain, "ConfigTerrain")};
      end

      terrain.kType_ = config_terrain.getSurfaceType();
      terrain.kInclination_ = config_terrain.getSurfaceInclination();

      terrain.loadSurfaceDataFromMatFile();
      terrain.setPointCloudInWorldFrame();
      terrain.setNormalVectors();

      terrain.setSurfaceCoefficients(config_terrain);

      terrain.graspable_points_ = GraspablePoints(config_terrain);
      terrain.graspable_points_.setGraspablePoints(terrain.kPointCloudInWorld_);

      [terrain.kGridColor_, terrain.kTransparency_] = config_terrain.getTerrainVisualSettings();
      terrain.createSurfaceGraphics();
    end

    function visualize(terrain, time)
      if (time ~= 0.0)
        return;
      end
      terrain.graphics_.Visible = "on";
    end

  end

  %% Private Methods
  methods (Access = private)

    function loadSurfaceDataFromMatFile(terrain)
    % Load surface points from .mat file
      folder = "src/environment/map";
      map_file_name = "map_" + terrain.kType_ + ".mat";
      file_path = fullfile(folder, map_file_name);
      if (~exist(file_path, "file"))
        error("Invalid surface type is specified" + newline ...
          + "Check ""type"" defined in config file.");
      end
      load(file_path, "x", "y", "z");
      terrain.kRawMapData_.x = x;  % 1 x n vector
      terrain.kRawMapData_.y = y;  % 1 x m vector
      terrain.kRawMapData_.z = z;  % m x n matrix

      dx = mean(diff(x));
      dy = mean(diff(y));
      if (dx <= dy)
        terrain.kPointDx_ = dx;
      else
        terrain.kPointDx_ = dy;
      end
    end

    function setPointCloudInWorldFrame(terrain)
    % Set map point data as point cloud described in the world frame
      x = terrain.kRawMapData_.x;
      y = terrain.kRawMapData_.y;
      z = terrain.kRawMapData_.z;
      point_cloud_in_Surface = [repelem(x, size(y, 2)); ...
                                repmat(y, 1, size(x, 2)); ...
                                reshape(z, 1, [])];
      % inclined_surface_point_cloud = ...
      %   eul2rotm(deg2rad(terrain.inclination)', "ZYX") * point_cloud_tmp;
      inclined_surface_point_cloud = rpy2dc(deg2rad(terrain.kInclination_))' * ...
        point_cloud_in_Surface;
      terrain.kPointCloudInWorld_ = [ inclined_surface_point_cloud(1, :); ...
                                      inclined_surface_point_cloud(2, :); ...
                                      inclined_surface_point_cloud(3, :)];
    end

    function setNormalVectors(terrain)
    % Calculate and set normal vectors at each points of terrain surface
      [Nx, Ny, Nz] = surfnorm(terrain.kRawMapData_.z);
      norm_vector_in_Surface = [reshape(Nx, 1, []); reshape(Ny, 1, []); reshape(Nz, 1, [])];
      terrain.kNormalVectors_ = rpy2dc(deg2rad(terrain.kInclination_))' * norm_vector_in_Surface;
    end

    function setSurfaceCoefficients(terrain, config_terrain)
    % Set contact characteristics (stiffness and damping)
      [Kf, Df, Km, Dm] = config_terrain.getGroundCoefficients();
      terrain.kStiffnessCoefficientForGRF_ = Kf;
      terrain.kDampingCoefficientForGRF_   = Df;
      terrain.kStiffnessCoefficientForGRM_ = Km;
      terrain.kDampingCoefficientForGRM_   = Dm;
    end

    function createSurfaceGraphics(terrain)
      [X, Y] = meshgrid(terrain.kRawMapData_.x, terrain.kRawMapData_.y);
      Z = terrain.kRawMapData_.z;
      map_vec(1, :) = reshape(X, 1, numel(X));
      map_vec(2, :) = reshape(Y, 1, numel(Y));
      map_vec(3, :) = reshape(Z, 1, numel(Z));

      % inclined_map_vec = eul2rotm(deg2rad(terrain.inclination)', "ZYX") * map_vec;
      inclined_map_vec = rpy2dc(deg2rad(terrain.kInclination_))' * map_vec;
      inclined_X = reshape(inclined_map_vec(1, :), size(X, 1), size(X, 2));
      inclined_Y = reshape(inclined_map_vec(2, :), size(Y, 1), size(Y, 2));
      inclined_Z = reshape(inclined_map_vec(3, :), size(Z, 1), size(Z, 2));

      terrain.graphics_ = mesh(inclined_X, inclined_Y, inclined_Z, ...
        EdgeColor = terrain.kGridColor_, ...
        EdgeAlpha = terrain.kTransparency_, ...
        Visible = "off");
    end

  end

  %% Getter
  methods (Access = public)
    function inclination = getSurfaceInclination(terrain)
      inclination = terrain.kInclination_;
    end

    function [Kf, Df, Km, Dm] = getGroundCoefficients(terrain)
      Kf = terrain.kStiffnessCoefficientForGRF_;
      Df = terrain.kDampingCoefficientForGRF_;
      Km = terrain.kStiffnessCoefficientForGRM_;
      Dm = terrain.kDampingCoefficientForGRM_;
    end

    function graspable_points = getGraspablePoints(terrain)
      graspable_points = terrain.graspable_points_;
    end

    function nearest_point = getNearestPointInWorldFrame(terrain, original_point)
    % getNearestPointInWorldFrame() Obtain the correspondent map positions
    %   Obtain the closest point in the map for a point
    %   HACK: Don't use point cloud to obtain the closest point for high computation speed
    %   Created:      2019.09.30 by Victoria Keo, Warley Ribeiro
    %   Last updated: 2024.03.15 by Masazumi Imai
    %
    % Input  - original_point (3x1): Given position of the point to be checked in the World frame
    % Output - nearest_point (3x1): Closest point positions for the map in the World frame
      arguments (Input)
        terrain;
        original_point (3, 1) {mustBeA(original_point, "double")};
      end

      original_point_in_Surface = rpy2dc(deg2rad(terrain.kInclination_)) * original_point;

      [~, id_x] = min(abs(terrain.kRawMapData_.x - original_point_in_Surface(1, 1)));
      nearest_point_in_Surface(1, 1) = terrain.kRawMapData_.x(1, id_x);

      [~, id_y] = min(abs(terrain.kRawMapData_.y - original_point_in_Surface(2, 1)));
      nearest_point_in_Surface(2, 1) = terrain.kRawMapData_.y(1, id_y);

      nearest_point_in_Surface(3, 1) = terrain.kRawMapData_.z(id_y, id_x);

      nearest_point = rpy2dc(deg2rad(terrain.kInclination_))' * nearest_point_in_Surface;
    end

    function norm_vector_at_point = getNormalVectorAtPoint(terrain, point)
      arguments (Input)
        terrain;
        point (3, 1) {mustBeA(point, "double")};
      end

      % HACK: [~, idx] = min(vecnorm(terrain.kPointCloudInWorld_ - point)); this takes longer time than the following one
      kDistThreshold = terrain.kPointDx_;
      [~, idx] = find(all(abs(terrain.kPointCloudInWorld_ - point) < kDistThreshold));
      if (length(idx) > 1)
        [~, idx_min] = min(vecnorm(terrain.kPointCloudInWorld_(:, idx) - point));
      else
        idx_min = 1;
      end

      norm_vector_at_point = terrain.kNormalVectors_(:, idx(idx_min));
    end
  end

end  % Terrain
