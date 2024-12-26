classdef ConfigEvaluation < Configuration
% Configuration for evaluation
%
% Created:      2020.07.08 by Warley Ribeiro
% Last updated: 2024.12.25 by Masazumi Imai

  %% Properties
  properties (SetAccess = {?ConfigEvaluation, ?Configuration}, GetAccess = public)
    evaluate_manipulability (1, 1) logical = true;
    evaluate_dynamic_manipulability (1, 1) logical = false;

    visualize_supporting_leg_polygon (1, 1) logical = false;
      supporting_leg_polygon_face_color = [0.0 136.0 / 255.0 170.0 / 255.0];
      supporting_leg_polygon_edge_color = "none";
      supporting_leg_polygon_face_transparency (1, 1) double = 0.5

    evaluate_tumble_stability_margin (1, 1) logical = false;
    evaluate_gravito_inertial_acceleration (1, 1) logical = false;
  end

  %% Constructor
  methods (Access = public)

    function config_evaluation = ConfigEvaluation(config)
    % ConfigJointController() Constructor
    %   Override properties value based on specified config file if config is not "default"
      arguments (Input)
        config (1, 1) {mustBeA(config, "string")};
      end

      if (config == "default")
        return;
      end

      config_evaluation = config_evaluation.override(config);
    end

  end

  %% Getter
  methods (Access = public)

    function [evaluate_manipulability, evaluate_dynamic_manipulability] = getEvaluateManipulabilities(config_evaluation)
      evaluate_manipulability = config_evaluation.evaluate_manipulability;
      evaluate_dynamic_manipulability = config_evaluation.evaluate_dynamic_manipulability;
    end

    function [visualize, face_color, edge_color, face_transparency] = getSupportingLegPolygonVisualSettings(config_evaluation)
      visualize = config_evaluation.visualize_supporting_leg_polygon;
      face_color = config_evaluation.supporting_leg_polygon_face_color;
      edge_color = config_evaluation.supporting_leg_polygon_edge_color;
      face_transparency = config_evaluation.supporting_leg_polygon_face_transparency;
    end

    function evaluate_tumble_stability_margin = getEvaluateTumbleStabilityMargin(config_evaluation)
      evaluate_tumble_stability_margin = config_evaluation.evaluate_tumble_stability_margin;
    end

    function evaluate_gravito_inertial_acceleration = getEvaluateGravitoInertialAcceleration(config_evaluation)
      evaluate_gravito_inertial_acceleration = config_evaluation.evaluate_gravito_inertial_acceleration;
    end

  end

end  % ConfigEvaluation
