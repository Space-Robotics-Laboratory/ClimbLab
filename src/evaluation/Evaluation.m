classdef Evaluation < handle
% Evaluation robot state
%
% Created     : 2020.04.23 by Warley Ribeiro
% Last updated: 2024.12.25 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    manipulability_ Manipulability;

    supporting_leg_polygon_ SupportingLegPolygon;
    tumble_stability_margin_ TumbleStabilityMargin;
    gravito_inertial_acceleration_ GravitoInertialAcceleration;
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
      evaluation.gravito_inertial_acceleration_ = GravitoInertialAcceleration(config_evaluation);
    end

    function evaluate(evaluation, world, terrain, robot)
      arguments (Input)
        evaluation;
        world   (1, 1) {mustBeA(world,   "World")};
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
        robot   (1, 1) {mustBeA(robot,   "Robot")};
      end

      evaluation.manipulability_.evaluate(robot);

      evaluation.supporting_leg_polygon_.calculate(robot);

      evaluation.tumble_stability_margin_.evaluate(world.getGravity(), terrain, ...
        robot.getLinkParameter(), robot.getStateVariable(), evaluation.supporting_leg_polygon_);

      evaluation.gravito_inertial_acceleration_.evaluate(world.getGravity(), robot.getLinkParameter(), robot.getStateVariable(), robot.getEEPosition());
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
