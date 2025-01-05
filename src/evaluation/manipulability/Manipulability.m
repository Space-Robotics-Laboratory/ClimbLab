classdef Manipulability < handle
% Calculate manipulability measure and dynamic manipulability measure
%
% Created     : 2020.04.10 by Koizumi Yusuke
% Last updated: 2024.12.18 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    kEvaluateManipulability_ (1, 1) logical;
    kEvaluateDynamicManipulability_ (1, 1) logical;
  end
  properties (SetAccess = private, GetAccess = public)
    manipulability_measure_ (:, 1) double;  % (kNumLimb x 1)
    dynamic_manipulability_measure_ (:, 1) double;  % (kNumLimb x 1)
  end

  %% Public Methods
  methods (Access = public)

    function manipulability = Manipulability(config_evaluation, robot)
      arguments (Input)
        config_evaluation (1, 1) {mustBeA(config_evaluation, "ConfigEvaluation")};
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      [manipulability.kEvaluateManipulability_, manipulability.kEvaluateDynamicManipulability_] = ...
        config_evaluation.getEvaluateManipulabilities();

      kNumLimb = robot.LP_.getNumberOfLimb();

      manipulability.manipulability_measure_ = zeros(kNumLimb, 1);
      manipulability.dynamic_manipulability_measure_ = zeros(kNumLimb, 1);
    end

    function evaluate(manipulability, robot)
      arguments (Input)
        manipulability;
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      LP = robot.getLinkParameter();
      SV = robot.getStateVariable();

      if (manipulability.kEvaluateManipulability_)
        manipulability.calcManipulability(LP, SV);
      end

      if (manipulability.kEvaluateDynamicManipulability_)
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

        manipulability.manipulability_measure_(limb_id, 1) = sqrt(det(Je * Je'));
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

        manipulability.dynamic_manipulability_measure_(limb_id, 1) = sqrt(det((Je / (hm' * hm) * Je')));
      end
    end

  end

  %% Getter
  methods (Access = public)

    function manipulability_measure = getManipulabilityMeasure(manipulability)
      manipulability_measure = manipulability.manipulability_measure_;
    end

    function dynamic_manipulability_measure = getDynamicManipulabilityMeasure(manipulability)
      dynamic_manipulability_measure = manipulability.dynamic_manipulability_measure_;
    end

  end

end  % Manipulability
