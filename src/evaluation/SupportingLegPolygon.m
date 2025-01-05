
classdef SupportingLegPolygon < handle
% Supporting Leg Polygon (with multiple triangles)
%
% Created     : 2024.12.21 by Masazumi Imai
% Last updated: 2024.12.24 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    % Combinations of limb IDs for possible forming supporting leg triangle (n x 3)
    kLimbIdsForPossibleSupportingLegTriangle_ (:, 3) uint8;

    is_forming_supporting_leg_triangle_ (:, 1) logical;  % (n x 1)

    % Vertices of supporting leg triangles (3 x 3 x n)
    %   1st dim: x-y-z coordinates
    %   2nd dim: Limb IDs for supporting leg triangle
    %   3rd dim: Supporting leg triangle number
    support_leg_triangles_ (3, 3, :) double;

    % Number of possible supporting leg triangles
    number_of_supporting_leg_triangles_ (1, 1) uint8;

    % Vertices of supporting leg polygon (3 x n)
    %   1st dim: x-y-z coordinates
    %   2nd dim: Number of vertices
    vertices_of_supporting_leg_polygon_ (3, :) double;

    graphics_ (:, 1) matlab.graphics.primitive.Patch;
  end
  properties (SetAccess = private, GetAccess = private)

    kVisualizeSupportingLegPolygon_ (1, 1) logical;
    kColor_;
    kEdgeColor_;
    kTransparency_ (1, 1) double;
    % Offset from actual vertices position so that support polygon should be drawn a bit higher not to be buried in the terrain surface visualization
    offset_ (3, 1) double = [0.0; 0.0; 0.015];
  end

  %% Public Methods
  methods (Access = public)

    function supporting_leg_polygon = SupportingLegPolygon(config_evaluation, terrain, robot)
    % Constructor
      arguments (Input)
        config_evaluation (1, 1) {mustBeA(config_evaluation, "ConfigEvaluation")};
        terrain           (1, 1) {mustBeA(terrain,           "Terrain")};
        robot             (1, 1) {mustBeA(robot,             "Robot")};
      end

      kNumLimb = robot.getLinkParameter().getNumberOfLimb();
      supporting_leg_polygon.kLimbIdsForPossibleSupportingLegTriangle_ = nchoosek(1 : kNumLimb, 3);
      supporting_leg_polygon.number_of_supporting_leg_triangles_ = size(supporting_leg_polygon.kLimbIdsForPossibleSupportingLegTriangle_, 1);

      supporting_leg_polygon.calcSupportingLegTriangle(robot.getStateVariable(), robot.getEEPosition());

      [supporting_leg_polygon.kVisualizeSupportingLegPolygon_, ...
        supporting_leg_polygon.kColor_, ...
        supporting_leg_polygon.kEdgeColor_, ...
        supporting_leg_polygon.kTransparency_] = ...
        config_evaluation.getSupportingLegPolygonVisualSettings();
      supporting_leg_polygon.createGraphics(terrain.getSurfaceInclination());
    end

    function calculate(supporting_leg_polygon, robot)
      arguments (Input)
        supporting_leg_polygon;
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      supporting_leg_polygon.calcSupportingLegTriangle(robot.getStateVariable(), robot.getEEPosition());
      supporting_leg_polygon.calcSupportingLegPolygon(robot.getLinkParameter(), robot.getStateVariable(), robot.getEEPosition())
    end

    function visualize(supporting_leg_polygon)
      if (~supporting_leg_polygon.kVisualizeSupportingLegPolygon_)
        return;
      end

      for i = 1 : supporting_leg_polygon.number_of_supporting_leg_triangles_
        if (supporting_leg_polygon.is_forming_supporting_leg_triangle_(i, 1))
          supporting_leg_polygon.graphics_(i, 1).XData = supporting_leg_polygon.support_leg_triangles_(1, :, i) + supporting_leg_polygon.offset_(1, 1);
          supporting_leg_polygon.graphics_(i, 1).YData = supporting_leg_polygon.support_leg_triangles_(2, :, i) + supporting_leg_polygon.offset_(2, 1);
          supporting_leg_polygon.graphics_(i, 1).ZData = supporting_leg_polygon.support_leg_triangles_(3, :, i) + supporting_leg_polygon.offset_(3, 1);

          supporting_leg_polygon.graphics_(i, 1).Visible = "on";
        else
          supporting_leg_polygon.graphics_(i, 1).Visible = "off";
        end
      end
    end

  end

  %% Private Methods
  methods (Access = private)

    function calcSupportingLegTriangle(supporting_leg_polygon, SV, EE_position)
    % Calculate supporting leg triangles
    %
    % Input - SV         : State variables
    %       - EE_position: End-effector position
      arguments (Input)
        supporting_leg_polygon;
        SV          (1, 1) {mustBeA(SV,          "StateVariable")};
        EE_position (3, :) {mustBeA(EE_position, "double")};
      end

      is_supporting = SV.getIsSupporting();
      for i = 1 : supporting_leg_polygon.number_of_supporting_leg_triangles_
        if (all(is_supporting(supporting_leg_polygon.kLimbIdsForPossibleSupportingLegTriangle_(i, :))))
          supporting_leg_polygon.is_forming_supporting_leg_triangle_(i, 1) = true;

          supporting_leg_polygon.support_leg_triangles_(:, :, i) = EE_position(:, supporting_leg_polygon.kLimbIdsForPossibleSupportingLegTriangle_(i, :));
        else
          supporting_leg_polygon.is_forming_supporting_leg_triangle_(i, 1) = false;
        end
      end
    end

    function calcSupportingLegPolygon(supporting_leg_polygon, LP, SV, EE_position)
      kNumLimb = LP.getNumberOfLimb();
      is_supporting = SV.getIsSupporting();
      vertices_of_supporting_leg_polygon = NaN(3, kNumLimb);

      for limb_id = 1 : kNumLimb
        if (is_supporting(1, limb_id))
          vertices_of_supporting_leg_polygon(:, limb_id) = EE_position(:, limb_id);
        end
      end

      supporting_leg_polygon.vertices_of_supporting_leg_polygon_ = vertices_of_supporting_leg_polygon;
    end

    function createGraphics(supporting_leg_polygon, surface_inclination)
    % Create graphics for each supporting leg triangles
    %
    % Input - surface_inclination: Inclination of terrain surface
      arguments (Input)
        supporting_leg_polygon;
        surface_inclination (3, 1) {mustBeA(surface_inclination, "double")};
      end

      supporting_leg_polygon.offset_ = rpy2dc(deg2rad(surface_inclination))' * supporting_leg_polygon.offset_;

      for i = 1 : supporting_leg_polygon.number_of_supporting_leg_triangles_
        p = supporting_leg_polygon.support_leg_triangles_(:, :, i) + supporting_leg_polygon.offset_;
        supporting_leg_polygon.graphics_(i, 1) = fill3(p(1, :), p(2, :), p(3, :), ...
          supporting_leg_polygon.kColor_, ...
          EdgeColor = supporting_leg_polygon.kEdgeColor_, ...
          FaceAlpha = supporting_leg_polygon.kTransparency_, ...
          Visible = "off");
      end
    end

  end

  %% Getter
  methods (Access = public)

    function vertices_of_supporting_leg_polygon_ =  getVerticesOfSupportingLegPolygon(supporting_leg_polygon)
      vertices_of_supporting_leg_polygon_ = supporting_leg_polygon.vertices_of_supporting_leg_polygon_;
    end

  end

end  % SupportingLegPolygon
