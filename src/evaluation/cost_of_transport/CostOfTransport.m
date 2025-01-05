classdef CostOfTransport < handle
% Compute Cost of Transport
%
% Created     : 2022.03.04 by Masazumi Imai
% Last updated: 2025.01.05 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    kEvaluateCostOfTransport_ (1, 1) logical;
  end
  properties (SetAccess = private, GetAccess = public)
    cost_of_transport_ (1, 1) double;
  end

  %% Public Methods
  methods (Access = public)

    function CoT = CostOfTransport(config_evaluation)
      arguments (Input)
        config_evaluation (1, 1) {mustBeA(config_evaluation, "ConfigEvaluation")};
      end

      CoT.kEvaluateCostOfTransport_ = config_evaluation.getEvaluateCostOfTransport();
    end

    function evaluate(CoT, kGravity, LP, SV)
    % Compute Cost of Transport based on the following equation:
    %
    %             n
    %            sum( |tau_i * qd_i|) )
    %            i=1
    %     CoT = ------------------------
    %                 m * g * v0
    %
    %       n  : Number of joints
    %       tau: Joint torque
    %       qd : Joint angular velocity
    %       m  : Total mass of robot
    %       g  : Gravity vector
    %       v0 : Locomotion speed
      arguments (Input)
        CoT;
        kGravity (3, 1) {mustBeA(kGravity, "double")};
        LP       (1, 1) {mustBeA(LP, "LinkParameters")};
        SV       (1, 1) {mustBeA(SV, "StateVariable")};
      end

      joint_torque = SV.getJointTorque();
      joint_angular_velocity = SV.getJointAngularVelocity();
      energy_per_joint = abs(joint_torque .* joint_angular_velocity);
      instantaneous_energy_consumption = sum(energy_per_joint, "all");

      kTotalMass = LP.getTotalMass();
      total_weight = kTotalMass * norm(kGravity);

      base_linear_velocity = SV.getBaseLinearVelocity();
      base_CoM_speed = norm(base_linear_velocity);

      CoT.cost_of_transport_ = instantaneous_energy_consumption / (total_weight * base_CoM_speed);
    end

  end

  %% Getter
  methods (Access = public)

    function cost_of_transport = getCoT(CoT)
      cost_of_transport = CoT.cost_of_transport_;
    end

  end

end  % CostOfTransport
