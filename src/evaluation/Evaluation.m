classdef Evaluation < handle

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    manipulability;

    tumble_stability_margin;
  end

  %% Public Methods
  methods (Access = public)

    function evaluation = Evaluation(config_evaluation, robot)
      arguments (Input)
        config_evaluation (1, 1) {mustBeA(config_evaluation, "ConfigEvaluation")};
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      evaluation.manipulability = Manipulability(config_evaluation, robot);

      evaluation.tumble_stability_margin = TumbleStabilityMargin();
    end

    function evaluate(evaluation, world, robot)
      evaluation.manipulability.evaluate(robot);

      evaluation.tumble_stability_margin.evaluate(world.getGravity(), robot.getLinkParameter(), robot.getStateVariable(), robot.getEEPosition());
    end

  end

end  % Evaluation
