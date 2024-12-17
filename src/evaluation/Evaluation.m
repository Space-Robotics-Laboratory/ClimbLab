classdef Evaluation < handle

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    manipulability;
  end

  %% Public Methods
  methods (Access = public)

    function evaluation = Evaluation(config_evaluation, robot)
      arguments (Input)
        config_evaluation (1, 1) {mustBeA(config_evaluation, "ConfigEvaluation")};
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      evaluation.manipulability = Manipulability(config_evaluation, robot);
    end

    function evaluate(evaluation, robot)
      evaluation.manipulability.evaluate(robot);
    end

  end

end  % Evaluation
