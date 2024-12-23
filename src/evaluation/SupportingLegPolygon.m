
classdef SupportingLegPolygon < handle

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    % Combinations of limb IDs for possible forming supporting leg triangle
    kLimbIdsForPossibleSupportingLegTriangle_;

    is_forming_supporting_leg_triangle_;
    % Vertices of support leg triangles (3 x 3 x n)
    %   1st dim: x-y-z coordinates
    %   2nd dim: Limb IDs for supporting leg triangle
    %   3rd dim: Supporting leg triangle number
    support_leg_triangles_;

    graphics_ (:, 1) matlab.graphics.primitive.Patch;
  end
  properties (SetAccess = private, GetAccess = private)
    number_of_supporting_leg_triangles_;

    kVisualizeSupportingLegPolygon_;
    kColor_;
    kTransparency_;
    kEdgeColor_;
  end

  %% Public Methods
  methods (Access = public)

    function supporting_leg_polygon = SupportingLegPolygon(config_evaluation, robot)
      kNumLimb = robot.getLinkParameter().getNumberOfLimb();
      supporting_leg_polygon.kLimbIdsForPossibleSupportingLegTriangle_ = nchoosek(1 : kNumLimb, 3);
      supporting_leg_polygon.number_of_supporting_leg_triangles_ = size(supporting_leg_polygon.kLimbIdsForPossibleSupportingLegTriangle_, 1);

      supporting_leg_polygon.calcSupportingLegTriangle(robot.getStateVariable(), robot.getEEPosition());

      [supporting_leg_polygon.kVisualizeSupportingLegPolygon_, ...
        supporting_leg_polygon.kColor_, ...
        supporting_leg_polygon.kEdgeColor_, ...
        supporting_leg_polygon.kTransparency_] = ...
        config_evaluation.getSupportingLegPolygonVisualSettings();
      supporting_leg_polygon.createGraphics();
    end

    function calculate(supporting_leg_polygon, robot)
      supporting_leg_polygon.calcSupportingLegTriangle(robot.getStateVariable(), robot.getEEPosition());
    end

    function visualize(supporting_leg_polygon)
      if (~supporting_leg_polygon.kVisualizeSupportingLegPolygon_)
        return;
      end

      for i = 1 : supporting_leg_polygon.number_of_supporting_leg_triangles_
        if (supporting_leg_polygon.is_forming_supporting_leg_triangle_(i, 1))
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

    function createGraphics(supporting_leg_polygon)
      for i = 1 : supporting_leg_polygon.number_of_supporting_leg_triangles_
        p = supporting_leg_polygon.support_leg_triangles_(:, :, i);
        supporting_leg_polygon.graphics_(i, 1) = fill3(p(1, :), p(2, :), p(3, :), ...
          supporting_leg_polygon.kColor_, ...
          EdgeColor = supporting_leg_polygon.kEdgeColor_, ...
          FaceAlpha = supporting_leg_polygon.kTransparency_, ...
          Visible = "off");
      end
    end

  end

end  % SupportingLegPolygon
