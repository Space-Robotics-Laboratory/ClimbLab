classdef GIAStabilityPolyhedron < handle
% Gravito-Inertial Acceleration Stability Polyhedron
%
% Created     : 2020.01.20 by Warley Ribeiro
% Last updated: 2025.01.03 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = private)
    kVisualizeGIAStableRegion_ (1, 1) logical;
    kFaceColor_
    kFaceTransparency_ (1, 1) double;
    kEdgeColor_;
    kEdgeWidth_ (1, 1) double;

    kAccelerationExpansionFactor_ (1, 1) double;
  end
  properties (SetAccess = private, GetAccess = public)
    % Maximum accelerations in a normal direction of tumbling axis [m/s^2] (3 x n matrix)
    plane_point_ (3, :) double;
    % Normal vector to tumbling axis from center of gravity (3 x n matrix)
    plane_vector_ (3, :) double;
    % Point in the limit plane based on robot position and the expansion factor (3 x n matrix)
    plane_point_exp_ (3, :) double;
    % Direction vector for the edges of the polyhedron [m] (3 x n matrix)
    edge_vector_ (3, :) double;
    % Position of a point in the line of the edge where the z position is null (3 x n matrix)
    edge_point_ (3, :) double;
    % Position of the corners of the polyhedron (3 x (n + 1) matrix)
    vertex_ (3, :) double;
  end
  properties (SetAccess = private, GetAccess = private)
    stable_region_graphics_ (:, 1) matlab.graphics.primitive.Patch;
  end

  %% Public Methods
  methods (Access = public)

    function GIA_stability_polyhedron = GIAStabilityPolyhedron(config_evaluation, animation)
    % Constructor
      arguments (Input)
        config_evaluation (1, 1) {mustBeA(config_evaluation, "ConfigEvaluation")};
        animation         (1, 1) {mustBeA(animation,         "Animation")};
      end

      [GIA_stability_polyhedron.kVisualizeGIAStableRegion_, ...
      GIA_stability_polyhedron.kFaceColor_, ...
      GIA_stability_polyhedron.kFaceTransparency_, ...
      GIA_stability_polyhedron.kEdgeColor_, ...
      GIA_stability_polyhedron.kEdgeWidth_] = config_evaluation.getGIAStableRegionVisualSettings();
      GIA_stability_polyhedron.kAccelerationExpansionFactor_ = animation.getAccelerationExpansionFactor();
    end

    function calcStableRegion(GIA_stability_polyhedron, p_g, end_effector_position, ...
        tumbling_axes, number_of_tumbling_axes, unit_normal_vector)
    % Calculate GIA stable region
    %
    % Input - p_g                    : Center of Gravity position [m] (3 x 1 vector)
    %       - end_effector_position  : End-effector positions (= [p_1, p_2, ... p_n]) [m] (3 x n matrix)
    %       - tumbling_axes          : Matrix with the number legs for tumbling axes (number_of_tumbling_axes x 2 matrix)
    %       - number_of_tumbling_axes: Total number of possible tumbling axis (scalar)
    %       - unit_normal_vector     : Unitary normal vector to the tumbling axis from CoG for all possible tumbling axes (3 x number_of_tumbling_axes matrix)
      arguments (Input)
        GIA_stability_polyhedron;
        p_g                     (3, 1) {mustBeA(p_g,                     "double")};
        end_effector_position   (3, :) {mustBeA(end_effector_position,   "double")};
        tumbling_axes           (:, 2) {mustBeA(tumbling_axes,           "uint8")};
        number_of_tumbling_axes (1, 1) {mustBeA(number_of_tumbling_axes, "uint8")};
        unit_normal_vector      (3, :) {mustBeA(unit_normal_vector,      "double")};
      end

      % To show stability polyhedron in cartesian space shrunk by a factor of "expansion_factor"
      expansion_factor = GIA_stability_polyhedron.kAccelerationExpansionFactor_;
      plane_point = GIA_stability_polyhedron.plane_point_;

      % Shrink vector and move to center of gravity
      plane_point_exp = expansion_factor * plane_point + p_g;
      GIA_stability_polyhedron.plane_point_exp_ = plane_point_exp;

      % Calculate intersection lines (Polyhedron Edges)
      kThreshold = 0.0001;
      edge_vector = zeros(3, number_of_tumbling_axes);
      edge_point = zeros(3, number_of_tumbling_axes);
      for i = 1 : number_of_tumbling_axes
        if (i == number_of_tumbling_axes)
          j = 1;
        else
          j = i + 1;
        end

        % Intersection line direction
        edge_vector(:, i) = cross(unit_normal_vector(:, i), unit_normal_vector(:, j));

        % Intersection line point (z = 0)
        if (abs(edge_vector(3, i)) > kThreshold)
          edge_point(3, i) = 0.0;
          A = [ unit_normal_vector(1, i), unit_normal_vector(2, i);
                unit_normal_vector(1, j), unit_normal_vector(2, j)];
          B = [ unit_normal_vector(:, i)' * plane_point_exp(:, i);
                unit_normal_vector(:, j)' * plane_point_exp(:, j)];
          edge_point(1 : 2, i) = A \ B;
        else
          % Intersection line point (y = 0)
          if (abs(edge_vector(2, i)) > kThreshold)
            edge_point(2, i) = 0.0;
            A = [ unit_normal_vector(1, i), unit_normal_vector(3, i);
                  unit_normal_vector(1, j), unit_normal_vector(3, j)];
            B = [ unit_normal_vector(:, i)' * plane_point_exp(:, i);
                  unit_normal_vector(:, j)' * plane_point_exp(:, j)];
            edge_point([1, 3], i) = A \ B;
          % Intersection line point (x = 0)
          else
            edge_point(1, i) = 0.0;
            A = [ unit_normal_vector(2, i), unit_normal_vector(3, i);
                  unit_normal_vector(2, j), unit_normal_vector(3, j)];
            B = [ unit_normal_vector(:, i)' * plane_point_exp(:, i);
                  unit_normal_vector(:, j)' * plane_point_exp(:, j)];
            edge_point(2 : 3, i) = A \ B;
          end
        end
      end
      GIA_stability_polyhedron.edge_vector_ = edge_vector;
      GIA_stability_polyhedron.edge_point_ = edge_point;

      % Calculate intersection points
      % Compute support triangle surface
      v_1 = end_effector_position(:, tumbling_axes(1, 1)) - end_effector_position(:, tumbling_axes(1, 2));
      v_2 = end_effector_position(:, tumbling_axes(2, 1)) - end_effector_position(:, tumbling_axes(2, 2));
      sup_plane_vector = cross(v_1, v_2);
      sup_plane_point  = end_effector_position(:, tumbling_axes(1, 1));
      vertex = zeros(3, number_of_tumbling_axes + 1);
      for k = 1 : number_of_tumbling_axes
        % Intersection between lines and support triangle surface
        if (abs(edge_vector(:, k)' * sup_plane_vector) > kThreshold)
          ind = (sup_plane_point - edge_point(:, k))' * sup_plane_vector / (edge_vector(:, k)' * sup_plane_vector);
        else
          disp("Polyhedron edge is parallel to surface. Unable to compute correct visualization");
          ind = 0.0;
        end
        vertex(:, k) = edge_point(:, k) + ind * edge_vector(:, k);
      end
      % Between lines
      C = edge_vector(:, 1) - edge_vector(:, 2);
      D = edge_point(:, 1) - edge_point(:, 2);
      ind = pinv(C) * D;
      vertex(:, end + 1) = edge_point(:, 1) - ind(1) * edge_vector(:, 1);
      GIA_stability_polyhedron.vertex_ = vertex;
    end

    function visualizeStableRegion(GIA_stability_polyhedron, terrain, number_of_tumbling_axes)
    % Visualize GIA stable region
    %
    % Input - terrain                : Terrain class
    %       - number_of_tumbling_axes: Total number of possible tumbling axis (scalar)
      arguments (Input)
        GIA_stability_polyhedron;
        terrain                 (1, 1) {mustBeA(terrain,                 "Terrain")};
        number_of_tumbling_axes (1, 1) {mustBeA(number_of_tumbling_axes, "uint8")};
      end

      if (~GIA_stability_polyhedron.kVisualizeGIAStableRegion_ || ...
          number_of_tumbling_axes <= 1)
        return;
      end

      surface_inclination = terrain.getSurfaceInclination();
      % Rotation matrix
      rot = rpy2dc(deg2rad(surface_inclination))';

      % Rotate polyhedron to match surface inclination
      polyhedron_vertex = rot' * GIA_stability_polyhedron.vertex_;

      for i = 1 : size(polyhedron_vertex, 2) - 1
        GIA_stability_polyhedron.stable_region_graphics_(i, 1) = patch( ...
          'Vertices', [polyhedron_vertex(:, i), polyhedron_vertex(:, i + 1), polyhedron_vertex(:, end)]', ...
          'Faces', [1, 2, 3], ...
          'FaceColor', GIA_stability_polyhedron.kFaceColor_, ...
          'EdgeColor', GIA_stability_polyhedron.kEdgeColor_, ...
          'FaceAlpha', GIA_stability_polyhedron.kFaceTransparency_, ...
          'LineWidth', GIA_stability_polyhedron.kEdgeWidth_);
      end
      GIA_stability_polyhedron.stable_region_graphics_(end + 1, 1) = patch( ...
        'Vertices', [polyhedron_vertex(:, end - 1), polyhedron_vertex(:, 1), polyhedron_vertex(:, end)]', ...
        'Faces', [1, 2, 3], ...
        'FaceColor', GIA_stability_polyhedron.kFaceColor_, ...
        'EdgeColor', GIA_stability_polyhedron.kEdgeColor_, ...
        'FaceAlpha', GIA_stability_polyhedron.kFaceTransparency_, ...
        'LineWidth', GIA_stability_polyhedron.kEdgeWidth_);
    end

    function resetStableRegion(GIA_stability_polyhedron)
    % Delete GIA stable region graphics
      if (GIA_stability_polyhedron.kVisualizeGIAStableRegion_)
        delete(GIA_stability_polyhedron.stable_region_graphics_);
      end
    end

  end

  %% Setter
  methods (Access = public)

    function setPlanePoint(GIA_stability_polyhedron, max_acceleration_in_normal_direction)
      GIA_stability_polyhedron.plane_point_ = max_acceleration_in_normal_direction;
    end

    function setPlaneVector(GIA_stability_polyhedron, normal_vector_to_tumbling_axis_from_CoG)
      GIA_stability_polyhedron.plane_vector_ = normal_vector_to_tumbling_axis_from_CoG;
    end

  end

  %% Getter
  methods (Access = public)

    function plane_point = getPlanePoint(GIA_stability_polyhedron)
      plane_point = GIA_stability_polyhedron.plane_point_;
    end

    function plane_vector = getPlaneVector(GIA_stability_polyhedron)
      plane_vector = GIA_stability_polyhedron.plane_vector_;
    end

  end

end  % GIAStabilityPolyhedron
