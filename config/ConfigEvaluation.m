classdef ConfigEvaluation < Configuration

  %% Properties
  properties (SetAccess = {?ConfigEvaluation, ?Configuration}, GetAccess = public)
    evaluate_manipulability (1, 1) logical = true;
    evaluate_dynamic_manipulability (1, 1) logical = false;

    evaluate_tumble_stability_margin (1, 1) logical = true;
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

    function evaluate_tumble_stability_margin = getEvaluateTumbleStabilityMargin(config_evaluation)
      evaluate_tumble_stability_margin = config_evaluation.evaluate_tumble_stability_margin;
    end

  end

end  % ConfigEvaluation
