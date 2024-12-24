classdef TumbleStabilityMargin < handle
% TumbleStabilityMargin
% Calculate Tumble Stability Margin
%
% Created     : 2020.04.23 by Warley Ribeiro
% Last updated: 2024.12.24 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    kEvaluateTumbleStabilityMargin_ (1, 1) logical;
  end
  properties (SetAccess = private, GetAccess = public)
    force_due_to_gravity_acceleration_  (3, 1);  % [N]
    moment_due_to_gravity_acceleration_ (3, 1);  % [Nm]

    force_due_to_inertial_acceleration_  (3, 1);  % [N]
    moment_due_to_inertial_acceleration_ (3, 1);  % [Nm]

    tumbling_axes_ (:, 2) uint8;  % TODO: This variable size is changed every time step. Need to modify
    number_of_tumbling_axes_ (1, 1) uint8;

    normal_vector_of_supporting_leg_polygon_plane_ (3, :) double;  % (3 x kNumLimb)

    % Tumbling moment for each tumbling axis (number_of_tumbling_axes_ x 1)  [Nm]
    tumbling_moment_ (:, 1) double;

    is_tumbling_ (:, 1) logical;  % Tumbling condition for each tumbling axis

    tumble_stability_margin_ (1, 1) double;  % [m]

    is_equilibrium_ (1, 1) logical;
  end

  %% Public Methods
  methods (Access = public)

    function TSM = TumbleStabilityMargin(config_evaluation)
    % Constructor
      arguments (Input)
        config_evaluation (1, 1) {mustBeA(config_evaluation, "ConfigEvaluation")};
      end

      TSM.kEvaluateTumbleStabilityMargin_ = config_evaluation.getEvaluateTumbleStabilityMargin();

      TSM.force_due_to_gravity_acceleration_ = zeros(3, 1);
      TSM.moment_due_to_gravity_acceleration_ = zeros(3, 1);

      TSM.force_due_to_inertial_acceleration_ = zeros(3, 1);
      TSM.moment_due_to_inertial_acceleration_ = zeros(3, 1);

      TSM.tumbling_axes_ = uint8.empty();
      TSM.number_of_tumbling_axes_ = 0;

      TSM.normal_vector_of_supporting_leg_polygon_plane_ = zeros(3, 1);

      TSM.tumbling_moment_ = 0.0;

      TSM.is_tumbling_ = false;

      TSM.tumble_stability_margin_ = 0.0;

      TSM.is_equilibrium_ = true;
    end

    function evaluate(TSM, gravity, terrain, LP, SV, supporting_leg_polygon)
      arguments (Input)
        TSM;
        gravity (3, 1) {mustBeA(gravity, "double")};
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
        LP (1, 1) {mustBeA(LP, "LinkParameters")};
        SV (1, 1) {mustBeA(SV, "StateVariable")};
        supporting_leg_polygon (1, 1) {mustBeA(supporting_leg_polygon, "SupportingLegPolygon")};
      end

      if (~TSM.kEvaluateTumbleStabilityMargin_)
        return;
      end

      TSM.calcGravitationalForceMoment(gravity, LP, SV);

      TSM.calcInertialForceMomentPlusRotational(LP, SV);

      TSM.calcTumblingAxes(LP, SV);

      TSM.calcTumblingMoment(terrain, LP, SV, supporting_leg_polygon);

      TSM.judgeEquilibrium(LP, SV, supporting_leg_polygon.getVerticesOfSupportingLegPolygon());

      TSM.calcTumbleStabilityMargin(gravity, LP)
    end

  end

  %% Private Methods
  methods (Access = private)

    function calcGravitationalForceMoment(TSM, gravity, LP, SV)
    % Obtain force and moment acting on the robot due to gravity acceleration at the CoM position
      arguments (Input)
        TSM;
        gravity (3, 1) {mustBeA(gravity, "double")};
        LP (1, 1) {mustBeA(LP, "LinkParameters")};
        SV (1, 1) {mustBeA(SV, "StateVariable")};
      end

      total_mass = LP.getTotalMass();
      CoM = SV.getCoM();

      % Force acting due to gravity acceleration
      TSM.force_due_to_gravity_acceleration_ = total_mass * gravity;
      % Moment acting due to gravity acceleration
      TSM.moment_due_to_gravity_acceleration_ = cross(CoM, total_mass * gravity);
    end

    function calcInertialForceMomentPlusRotational(TSM, LP, SV)
    % Obtain force and moment acting on the robot due to inertial acceleration, including angular
    % velocity and acceleration effects of each link
      arguments (Input)
        TSM;
        LP (1, 1) {mustBeA(LP, "LinkParameters")};
        SV (1, 1) {mustBeA(SV, "StateVariable")};
      end

      kNumJoints = LP.getNumberOfJoints();

      kBaseMass = LP.getBaseMass();
      base_inertia = LP.getBaseInertia();
      base_position = SV.getBasePosition();
      base_linear_acceleration = SV.getBaseLinearAcceleration();
      base_angular_velocity = SV.getBaseAngularVelocity();
      base_angular_acceleration = SV.getBaseAngularAcceleration();
      kLinksMass = LP.getLinksMass();
      links_inertia = LP.getLinksInertia();
      links_position = SV.getLinksPosition();
      links_angular_velocity = SV.getLinksAngularVelocity();
      links_linear_acceleration = SV.getLinksLinearAcceleration();
      links_angular_acceleration = SV.getLinksAngularAcceleration();

      % Force due to each link linear acceleration
      force_due_to_base_linear_acceleration = kBaseMass * base_linear_acceleration;
      force_due_to_link_linear_acceleration = zeros(3, kNumJoints);
      for link_id = 1 : kNumJoints
        force_due_to_link_linear_acceleration(:, link_id) = kLinksMass(1, link_id) * links_linear_acceleration(:, link_id);
      end
      TSM.force_due_to_inertial_acceleration_ = force_due_to_base_linear_acceleration + sum(force_due_to_link_linear_acceleration, 2);

      % Moment due to each link linear acceleration
      moment_due_to_base_linear_acceleration = cross(kBaseMass * base_position, base_linear_acceleration);
      moment_due_to_link_linear_acceleration = zeros(3, kNumJoints);
      for link_id = 1 : kNumJoints
        moment_due_to_link_linear_acceleration(:, link_id) = cross(kLinksMass(1, link_id) * links_position(:, link_id), links_linear_acceleration(:, link_id));
      end
      moment_due_to_inertial_linear_acceleration = moment_due_to_base_linear_acceleration + sum(moment_due_to_link_linear_acceleration, 2);

      % Moment due to each link angular acceleration
      moment_due_to_base_angular_acceleration = base_inertia * base_angular_acceleration + cross(base_angular_velocity, base_inertia * base_angular_velocity);
      moment_due_to_link_angular_acceleration = zeros(3, kNumJoints);
      for link_id = 1 : kNumJoints
        moment_due_to_link_angular_acceleration(:, link_id) = links_inertia(:, 3 * link_id - 2 : 3 * link_id) * links_angular_acceleration(:, link_id) + ...
          cross(links_angular_velocity(:, link_id), links_inertia(:, 3 * link_id - 2 : 3 * link_id) * links_angular_velocity(:, link_id));
      end
      moment_due_to_inertial_angular_acceleration = moment_due_to_base_angular_acceleration + sum(moment_due_to_link_angular_acceleration, 2);

      TSM.moment_due_to_inertial_acceleration_ = moment_due_to_inertial_linear_acceleration + moment_due_to_inertial_angular_acceleration;
    end

    function calcTumblingAxes(TSM, LP, SV)
    % Obtain limb IDs for all possible tumbling axes and total number of tumbling axes
      arguments (Input)
        TSM;
        LP (1, 1) {mustBeA(LP, "LinkParameters")};
        SV (1, 1) {mustBeA(SV, "StateVariable")};
      end

      kNumLimb = LP.getNumberOfLimb();
      is_supporting = SV.getIsSupporting();

      tumbling_axes = [];

      % TODO: Modify this code
      if (sum(is_supporting) > 1)

        cnt = 1;
        for i = 1 : kNumLimb
          if (is_supporting(1, i))
            tumbling_axes(cnt, 1) = i;

            for j = i + 1 : kNumLimb
              if (is_supporting(1, j))
                tumbling_axes(cnt, 2) = j;
                cnt = cnt + 1;
                break;
              end
            end
          end
        end

        % Close polygon with initial support point
        tumbling_axes(cnt, 2) = tumbling_axes(1, 1);

        % Total number of tumbling axes
        tumbling_axes_number = cnt;

      else

        % One or none supporting legs case
        for i = 1 : kNumLimb
          if (is_supporting(1, i))
            tumbling_axes(1, :) = [i, i];
          end
        end
        tumbling_axes_number = 0;

      end

      TSM.tumbling_axes_ = tumbling_axes;
      TSM.number_of_tumbling_axes_ = tumbling_axes_number;
    end

    function calcTumblingMoment(TSM, terrain, LP, SV, supporting_leg_polygon)
    % Calculate tumbling moment for tumbling axes
      arguments (Input)
        TSM;
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
        LP (1, 1) {mustBeA(LP, "LinkParameters")};
        SV (1, 1) {mustBeA(SV, "StateVariable")};
        supporting_leg_polygon (1, 1) {mustBeA(supporting_leg_polygon, "SupportingLegPolygon")};
      end

      kNumLimb = LP.getNumberOfLimb();
      is_supporting = SV.getIsSupporting();

      % Vertices of supporting leg polygon (supporting end-effector positions)
      vertices_of_supporting_leg_polygon = supporting_leg_polygon.getVerticesOfSupportingLegPolygon();

      % Calculate the normal vector of the support triangle plane
      normal_vectors = NaN(3, kNumLimb);
      for limb_id = 1 : kNumLimb
        if (is_supporting(1, limb_id))
          normal_vectors(:, limb_id) = terrain.getNormVectorAtPoint(vertices_of_supporting_leg_polygon(:, limb_id));
        end
      end
      TSM.normal_vector_of_supporting_leg_polygon_plane_ = normal_vectors;

      % Tumbling Moment Calculation
      if (TSM.number_of_tumbling_axes_ == 0)
        TSM.tumbling_moment_ = 0.0;
        return;
      end

      % Force due to inertial and gravitational acceleration
      F_bar = TSM.force_due_to_inertial_acceleration_ - TSM.force_due_to_gravity_acceleration_;
      % Moment due to inertial and gravitational acceleration
      M_bar = TSM.moment_due_to_inertial_acceleration_ - TSM.moment_due_to_gravity_acceleration_;

      for tumbling_axis_id = 1 : TSM.number_of_tumbling_axes_
        % Limb ID for tumbling axis
        limb_a = TSM.tumbling_axes_(tumbling_axis_id, 1);
        limb_b = TSM.tumbling_axes_(tumbling_axis_id, 2);
        % End-effector position of limb for tumbling axis
        p_a = vertices_of_supporting_leg_polygon(:, limb_a);
        p_b = vertices_of_supporting_leg_polygon(:, limb_b);

        % Moment due to inertial and gravitational forces around tumbling axis
        TSM.tumbling_moment_(tumbling_axis_id, 1) = ...
          M_bar' * (p_a - p_b) / abs(norm(p_a - p_b)) + ...
          F_bar' * cross(p_b, p_a) / abs(norm(p_a - p_b));

        if (LP.getMaxEndurableGrippingForce() == 0.0)
          continue;
        end

        % Check all possible gripping points besides the ones forming the tumbling axis
        for limb_id = 1 : kNumLimb
          if (limb_id ~= limb_a && limb_id ~= limb_b && is_supporting(1, limb_id))
            p_j = vertices_of_supporting_leg_polygon(:, limb_id);
            % Normal vector of terrain surface at p_j
            n_j = normal_vectors(:, limb_id);

            % Gripping force direction
            n_g = (cross((p_a - p_j), (p_b - p_j))) / abs(norm(cross((p_a - p_j), (p_b - p_j))));

            if (n_g' * n_j > 0.0)
              sign_gripping_force = -1;
            elseif (n_g' * n_j < 0.0)
              sign_gripping_force = 1;
            else
              sign_gripping_force = 0;
            end
            F_gripper = sign_gripping_force * LP.getMaxEndurableGrippingForce() * n_g;

            % Moment due to gripping force
            M_gripper = F_gripper' * cross(p_b - p_j, p_a - p_j) / abs(norm(p_a - p_b));

            % Update tumbling moment with gripping force
            TSM.tumbling_moment_(tumbling_axis_id, 1) = TSM.tumbling_moment_(tumbling_axis_id, 1) - M_gripper;
          end
        end
      end
    end

    function judgeEquilibrium(TSM, LP, SV, EE_position)
    % Judgment of equilibrium based on tumbling moment for each tumbling axis
      arguments (Input)
        TSM;
        LP (1, 1) {mustBeA(LP, "LinkParameters")};
        SV (1, 1) {mustBeA(SV, "StateVariable")};
        EE_position (3, :) {mustBeA(EE_position, "double")};
      end

      if (TSM.number_of_tumbling_axes_ == 0)
        TSM.is_tumbling_ = true;
        return;
      end

      TSM.is_tumbling_ = true(TSM.number_of_tumbling_axes_, 1);

      kNumLimb = LP.getNumberOfLimb();
      is_supporting = SV.getIsSupporting();
      normal_vectors = TSM.normal_vector_of_supporting_leg_polygon_plane_;
      tumbling_moment = TSM.tumbling_moment_;

      for tumbling_axis_id = 1 : TSM.number_of_tumbling_axes_
        % Limb ID for tumbling axis
        limb_a = TSM.tumbling_axes_(tumbling_axis_id, 1);
        limb_b = TSM.tumbling_axes_(tumbling_axis_id, 2);
        % End-effector position of limb for tumbling axis
        p_a = EE_position(:, limb_a);
        p_b = EE_position(:, limb_b);

        for limb_id = 1 : kNumLimb
          if (limb_id ~= limb_a && limb_id ~= limb_b && is_supporting(1, limb_id))
            p_j = EE_position(:, limb_id);
            n_j = normal_vectors(:, limb_id);

            not_tumbling_condition = cross(p_j - p_a, n_j)' * (tumbling_moment(tumbling_axis_id, 1) * (p_a - p_b) / abs(norm(p_a - p_b)));

            if (not_tumbling_condition > 0.0)
              TSM.is_tumbling_(tumbling_axis_id, 1) = false;
            end
          end
        end
      end
    end

    function calcTumbleStabilityMargin(TSM, gravity, LP)
      arguments (Input)
        TSM;
        gravity (3, 1) {mustBeA(gravity, "double")};
        LP      (1, 1) {mustBeA(LP,      "LinkParameters")};
      end

      total_mass = LP.getTotalMass();
      tumbling_moment = TSM.tumbling_moment_;

      % If a tumbling axis was judged as tumbling, tumbling moment is zero
      tumbling_moment(TSM.is_tumbling_, 1) = 0.0;

      % Tumble Stability Margin (TSM) [m]
      TSM.tumble_stability_margin_ = min(abs(tumbling_moment)) / abs(total_mass * norm(gravity));

      if (TSM.tumble_stability_margin_ == 0.0)
        TSM.is_equilibrium_ = false;
      else
        TSM.is_equilibrium_ = true;
      end
    end

  end

  %% Getter
  methods (Access = public)

    function tumble_stability_margin = getTumbleStabilityMargin(TSM)
      tumble_stability_margin = TSM.tumble_stability_margin_;
    end

  end

end  % TumbleStabilityMargin
