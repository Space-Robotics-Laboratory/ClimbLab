classdef Evaluation < handle

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    manipulability_ Manipulability;

    supporting_leg_polygon_ SupportingLegPolygon;
    tumble_stability_margin_ TumbleStabilityMargin;
  end

  %% Public Methods
  methods (Access = public)

    function evaluation = Evaluation(config_evaluation, terrain, robot)
      arguments (Input)
        config_evaluation (1, 1) {mustBeA(config_evaluation, "ConfigEvaluation")};
        terrain           (1, 1) {mustBeA(terrain,           "Terrain")};
        robot             (1, 1) {mustBeA(robot,             "Robot")};
      end

      evaluation.manipulability_ = Manipulability(config_evaluation, robot);

      evaluation.supporting_leg_polygon_ = SupportingLegPolygon(config_evaluation, terrain, robot);
      evaluation.tumble_stability_margin_ = TumbleStabilityMargin(config_evaluation);
    end

    function evaluate(evaluation, world, robot)
      evaluation.manipulability_.evaluate(robot);

      evaluation.supporting_leg_polygon_.calculate(robot);

      evaluation.tumble_stability_margin_.evaluate(world.getGravity(), robot.getLinkParameter(), robot.getStateVariable(), robot.getEEPosition());
    end

    function visualize(evaluation)
      evaluation.supporting_leg_polygon_.visualize();
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
