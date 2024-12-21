classdef Evaluation < handle

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    manipulability_ Manipulability;

    tumble_stability_margin_ TumbleStabilityMargin;
  end

  %% Public Methods
  methods (Access = public)

    function evaluation = Evaluation(config_evaluation, robot)
      arguments (Input)
        config_evaluation (1, 1) {mustBeA(config_evaluation, "ConfigEvaluation")};
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      evaluation.manipulability_ = Manipulability(config_evaluation, robot);

      evaluation.tumble_stability_margin_ = TumbleStabilityMargin(config_evaluation);
    end

    function evaluate(evaluation, world, robot)
      evaluation.manipulability_.evaluate(robot);

      evaluation.tumble_stability_margin_.evaluate(world.getGravity(), robot.getLinkParameter(), robot.getStateVariable(), robot.getEEPosition());
    end

  end

  %% Getter
  methods (Access = public)

    function manipulability = getManipulability(evaluation)
      manipulability = evaluation.manipulability_;
    end

    function TSM = getTumbleStabilityMargin(evaluation)
      TSM = evaluation.tumble_stability_margin_;
    end

  end

end  % Evaluation
