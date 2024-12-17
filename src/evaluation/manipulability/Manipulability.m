classdef Manipulability < handle
% Calculate manipulability measure and dynamic manipulability measure
%
% Created     : 2020.04.10 by Koizumi Yusuke
% Last updated: 2024.06.03 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    kEvaluateManipulability (1, 1) logical;
    manipulability_measure (:, 1) double;  % (kNumLimb x 1)

    kEvaluateDynamicManipulability (1, 1) logical;
    dynamic_manipulability_measure (:, 1) double;  % (kNumLimb x 1)
  end

  %% Public Methods
  methods (Access = public)

    function manipulability = Manipulability(config_evaluation, robot)
      arguments (Input)
        config_evaluation (1, 1) {mustBeA(config_evaluation, "ConfigEvaluation")};
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      [manipulability.kEvaluateManipulability, manipulability.kEvaluateDynamicManipulability] = ...
        config_evaluation.getEvaluateManipulabilities();

      kNumLimb = robot.LP_.getNumberOfLimb();

      manipulability.manipulability_measure = zeros(kNumLimb, 1);
      manipulability.dynamic_manipulability_measure = zeros(kNumLimb, 1);
    end

    function evaluate(manipulability, robot)
      arguments (Input)
        manipulability;
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      LP = robot.getLinkParameter();
      SV = robot.getStateVariable();

      if (manipulability.kEvaluateManipulability)
        manipulability.calcManipulability(LP, SV);
      end

      if (manipulability.kEvaluateDynamicManipulability)
        manipulability.calcDynamicManipulability(LP, SV);
      end
    end

  end

  %% Private Methods
  methods (Access = private)

    function calcManipulability(manipulability, LP, SV)
      kNumLimb = LP.getNumberOfLimb();
      kNumJointsPerLimb = LP.getNumberOfJointsPerLimb();
      kJoints = LP.getJoints();

      for limb_id = 1 : kNumLimb
        j = kNumJointsPerLimb(1, limb_id);
        Jacobian = calc_je(LP, SV, kJoints(:, limb_id));
        Je = Jacobian(1 : 3, j * (limb_id - 1) + 1 : j * limb_id);

        manipulability.manipulability_measure(limb_id, 1) = sqrt(det(Je * Je'));
      end
    end

    function calcDynamicManipulability(manipulability, LP, SV)
      kNumLimb = LP.getNumberOfLimb();
      kNumJointsPerLimb = LP.getNumberOfJointsPerLimb();
      kJoints = LP.getJoints();
      HH = calc_hh(LP, SV);
      Hm = HH(7 : end, 7 : end);

      for limb_id = 1 : kNumLimb
        j = kNumJointsPerLimb(1, limb_id);
        Jacobian = calc_je(LP, SV, kJoints(:, limb_id));
        Je = Jacobian(1 : 3, j * (limb_id - 1) + 1 : j * limb_id);
        hm = Hm(kJoints(:, limb_id), kJoints(:, limb_id));

        manipulability.dynamic_manipulability_measure(limb_id, 1) = sqrt(det((Je / (hm' * hm) * Je')));
      end
    end

  end

end  % Manipulability
