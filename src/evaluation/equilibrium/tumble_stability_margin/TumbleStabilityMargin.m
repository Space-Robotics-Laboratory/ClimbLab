classdef TumbleStabilityMargin < handle
% TumbleStabilityMargin
% Calculate Tumble Stability Margin
%
% Created     : 2020.04.23 by Warley Ribeiro
% Last updated: 2024.12.18 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
  end
  properties (SetAccess = private, GetAccess = private)
    force_due_to_gravity_acceleration_  (3, 1);  % [N]
    moment_due_to_gravity_acceleration_ (3, 1);  % [Nm]

    force_due_to_inertial_acceleration_  (3, 1);  % [N]
    moment_due_to_inertial_acceleration_ (3, 1);  % [Nm]

    tumbling_axes_ (:, 2) uint8;  % TODO: This variable size is changed every time step. Need to modify
    tumbling_axes_number_ (1, 1) uint8;  % TODO: Rename to number_of_tumbling_axes_
  end

  %% Public Methods
  methods (Access = public)

    function TSM = TumbleStabilityMargin()
    % TumbleStabilityMargin() Constructor
    end

    function calculate(TSM)
      TSM.calcGravitationalForceMoment(gravity, LP, SV);

      TSM.calcInertialForceMomentPlusRotational(LP, SV);

      TSM.calcTumblingAxes(LP, SV);
    end

  end

  %% Private Methods
  methods (Access = private)

    function calcGravitationalForceMoment(TSM, gravity, LP, SV)
    % calcGravitationalForceMoment()
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
    % calcInertialForceMomentPlusRotational()
    %   Obtain force and moment acting on the robot due to inertial acceleration, including angular
    %   velocity and acceleration effects of each link
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
      force_due_to_link_linear_acceleration = zeros(kNumJoints, 1);
      for link_id = 1 : kNumJoints
        force_due_to_link_linear_acceleration(link_id, 1) = kLinksMass(1, link_id) * links_linear_acceleration(:, link_id);
      end
      TSM.force_due_to_inertial_acceleration_ = force_due_to_base_linear_acceleration + sum(force_due_to_link_linear_acceleration);

      % Moment due to each link linear acceleration
      moment_due_to_base_linear_acceleration = cross(kBaseMass * base_position, base_linear_acceleration);
      moment_due_to_link_linear_acceleration = zeros(kNumJoints, 1);
      for link_id = 1 : kNumJoints
        moment_due_to_link_linear_acceleration(link_id, 1) = cross(kLinksMass(1, link_id) * links_position(:, link_id), links_linear_acceleration(:, link_id));
      end
      moment_due_to_inertial_linear_acceleration = moment_due_to_base_linear_acceleration + sum(moment_due_to_link_linear_acceleration);

      % Moment due to each link angular acceleration
      moment_due_to_base_angular_acceleration = base_inertia * base_angular_acceleration + cross(base_angular_velocity, base_inertia * base_angular_velocity);
      moment_due_to_link_angular_acceleration = zeros(kNumJoints, 1);
      for link_id = 1 : kNumJoints
        moment_due_to_link_angular_acceleration(link_id, 1) = links_inertia(:, 3 * link_id - 2 : 3 * link_id) * links_angular_acceleration(:, link_id) + ...
          cross(links_angular_velocity(:, link_id), links_inertia(:, 3 * link_id - 2 : 3 * link_id) * links_angular_velocity(:, link_id));
      end
      moment_due_to_inertial_angular_acceleration = moment_due_to_base_angular_acceleration * sum(moment_due_to_link_angular_acceleration);

      TSM.moment_due_to_inertial_acceleration_ = moment_due_to_inertial_linear_acceleration + moment_due_to_inertial_angular_acceleration;
    end

    function calcTumblingAxes(TSM, LP, SV)
    % calcTumblingAxes()
    %   Obtain limb IDs for all possible tumbling axes and total number of tumbling axes
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
            tumbling_axes(1, 1) = [i, i];
          end
        end
        tumbling_axes_number = 0;

      end

      TSM.tumbling_axes_ = tumbling_axes;
      TSM.tumbling_axes_number_ = tumbling_axes_number;
    end

    function calcTumblingMoment(TSM)
    end

    function judgeEquilibrium(TSM)
    end

    function calcTumbleStabilityMargin(TSM)
    end

  end

end  % TumbleStabilityMargin
