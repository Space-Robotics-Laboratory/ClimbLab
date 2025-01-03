classdef GravitoInertialAcceleration < handle
% Gravito-Inertial Acceleration
%
% Created     : 2020.04.23 by Warley Ribeiro
% Last updated: 2025.01.03 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    kEvaluateGravitoInertialAcceleration_ (1, 1) logical;
  end
  properties (SetAccess = private, GetAccess = public)
    % Gravito-Inertial Acceleration (g-a) vector [m/s^2]
    gia_vector_ (3, 1) double;

    is_equilibrium_ (1, 1) logical;

    stability_polyhedron_ GIAStabilityPolyhedron;

    % Acceleration margin considering the stability polyhedron [m/s^2] (scalar)
    gia_margin_ (1, 1) double;
    % Acceleration margin for each tumbling axis [m/s^2] (1 x n vector)
    gia_margin_for_each_tumbling_axis_ (1, :) double;

    % Inclination margin for total acceleration considering the support polyhedron [deg] (scalar)
    gia_inclination_margin_ (1, 1) double;
    % Inclination margin for each tumbling axis [rad] (1 x n vector)  % ?: [deg]?
    gia_inclination_margin_for_each_tumbling_axis_ (1, :) double;
  end
  properties (SetAccess = private, GetAccess = private)
    force_due_to_inertial_acceleration_  (3, 1) double;  % [N]  (F_alpha)
    moment_due_to_inertial_acceleration_ (3, 1) double;  % [Nm] (M_alpha)

    % Matrix with the number legs for tumbling axes (number_of_tumbling_axes x 2 matrix).
    % Each row represents one tumbling axis, while the columns represent the number of the leg for that specific axis
    tumbling_axes_ (:, 2) uint8;  % TODO: This variable size is changed every time step. Need to modify
    % Total number of possible tumbling axis (scalar)
    number_of_tumbling_axes_ (1, 1) uint8;

    % Normal vector to the tumbling axis from CoG for all possible tumbling axes (3 x number_of_tumbling_axes matrix)
    normal_vector_ (3, :) double;
    % Unitary normal vector to the tumbling axis from CoG for all possible tumbling axes (3 x number_of_tumbling_axes matrix)
    unit_normal_vector_ (3, :) double;

    % Acceleration limit for all possible tumbling axis faces of the equilibrium polyhedron (3 x number_of_tumbling_axes matrix)
    max_acceleration_in_normal_direction_
  end
  properties (SetAccess = immutable, GetAccess = private)
    kVisualizeGIAVector_ (1, 1) logical;
    kGIAVectorColor_;
    kGIAVectorWidth_ (1, 1) double;
  end

  %% Public Methods
  methods (Access = public)

    function GIA = GravitoInertialAcceleration(config_evaluation, animation)
    % Constructor
      arguments (Input)
        config_evaluation (1, 1) {mustBeA(config_evaluation, "ConfigEvaluation")};
        animation         (1, 1) {mustBeA(animation,         "Animation")};
      end

      GIA.kEvaluateGravitoInertialAcceleration_ = config_evaluation.getEvaluateGravitoInertialAcceleration();
      [GIA.kVisualizeGIAVector_, GIA.kGIAVectorColor_, GIA.kGIAVectorWidth_] = config_evaluation.getGIAVectorVisualSettings();

      GIA.force_due_to_inertial_acceleration_ = zeros(3, 1);
      GIA.moment_due_to_inertial_acceleration_ = zeros(3, 1);
      GIA.number_of_tumbling_axes_ = 0;
      GIA.gia_vector_ = zeros(3, 1);
      GIA.is_equilibrium_ = false;
      GIA.stability_polyhedron_ = GIAStabilityPolyhedron(config_evaluation, animation);
      GIA.gia_margin_ = 0.0;
      GIA.gia_margin_for_each_tumbling_axis_ = zeros(1, GIA.number_of_tumbling_axes_);
      GIA.gia_inclination_margin_ = 0.0;
      GIA.gia_inclination_margin_for_each_tumbling_axis_ = zeros(1, GIA.number_of_tumbling_axes_);
    end

    function evaluate(GIA, gravity, LP, SV, end_effector_position)
      arguments (Input)
        GIA;
        gravity (3, 1) {mustBeA(gravity, "double")};
        LP (1, 1) {mustBeA(LP, "LinkParameters")};
        SV (1, 1) {mustBeA(SV, "StateVariable")};
        end_effector_position (3, :) {mustBeA(end_effector_position, "double")};
      end

      if (~GIA.kEvaluateGravitoInertialAcceleration_)
        return;
      end

      GIA.calcInertialForceMomentPlusRotational(LP, SV);

      GIA.calcStabilityPolyhedron(gravity, LP, SV, end_effector_position);  % ?: should be implemented in GIAStabilityPolyhedron

      GIA.calcGIAAccelerationMargin();
      GIA.calcGIAInclinationMargin();
    end

    function visualize(GIA, terrain, robot, animation)
    % Visualize GIA vector and GIA stable region
      arguments (Input)
        GIA;
        terrain   (1, 1) {mustBeA(terrain,   "Terrain")};
        robot     (1, 1) {mustBeA(robot,     "Robot")};
        animation (1, 1) {mustBeA(animation, "Animation")};
      end

      GIA.visualizeGIAVector(robot, animation);
      GIA.stability_polyhedron_.visualizeStableRegion(terrain, GIA.number_of_tumbling_axes_);
    end

  end

  %% Private Methods
  methods (Access = private)

    function calcInertialForceMomentPlusRotational(GIA, LP, SV)
    % Obtain force and moment acting on the robot due to inertial acceleration, including angular
    % velocity and acceleration effects of each link
      arguments (Input)
        GIA;
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
      GIA.force_due_to_inertial_acceleration_ = force_due_to_base_linear_acceleration + sum(force_due_to_link_linear_acceleration, 2);

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

      GIA.moment_due_to_inertial_acceleration_ = moment_due_to_inertial_linear_acceleration + moment_due_to_inertial_angular_acceleration;
    end

    function calcStabilityPolyhedron(GIA, gravity, LP, SV, end_effector_position)
    % Calculate Gravito-Inertial Acceleration Stability Polyhedron
    % Considering the tumble stability with the addition of gripping forces to prevent the tumbling motion, the following equation describe the limit condition for one tumbling axis
    %
    %   m.a_gi.{(pg - pa) x (pg - pb)} = sum{Fj.{(pb - pj) x (pa - pj)}} + M0.(pa - pb) + F0.(pb x pa)
    %
    % If (g-a) is considered as a variable (the Gravito-inertial acceleration), the equation for the limit of the equilibrium
    % relative to a tumbling axis is a plane in the three-dimensional cartesian space. The intersection of planes for all the
    % possible tumbling axes gives a convex polyhedron. This is the stability polyhedron for the gravito-inertial acceleration,
    % which can used to define if a robot is in dynamic equilibrium or not, if the GIA vector (g-a) is inside the polyhedron.
    %     m   : Total mass of the robot
    %     a_gi: Gravito-inertial acceleration
    %     pa  : Position of the first point of the tumbling axis
    %     pb  : Position of the second point of the tumbling axis
    %     pg  : Position of the center of gravity
    %     pj  : Position of the other grasping points
    %     Fj  : Maximum holding force for the position j
    %     M0  : External moment applied to the center of gravity
    %     F0  : External force applied to the center of gravity
      arguments (Input)
        GIA;
        gravity (3, 1) {mustBeA(gravity, "double")};
        LP (1, 1) {mustBeA(LP, "LinkParameters")};
        SV (1, 1) {mustBeA(SV, "StateVariable")};
        end_effector_position (3, :) {mustBeA(end_effector_position, "double")};
      end

      F_grip = LP.getMaxEndurableGrippingForce();

      kNumLimb = LP.getNumberOfLimb();
      is_supporting = SV.getIsSupporting();  % ?: is_grasping = SV.getIsGrasping();?
      GIA.calcTumblingAxis(kNumLimb, is_supporting);

      CoM = SV.getCoM();
      GIA.calcNormalVector(CoM, end_effector_position);

      mass = LP.getTotalMass();  % [kg]
      external_force = [0.0; 0.0; 0.0];  % [N] (F_0)
      external_moment = [0.0; 0.0; 0.0];  % [Nm] (M_0)
      GIA.calcMaximumAccelerationForNormalVectorDirection(...
        mass, external_force, external_moment, end_effector_position, is_supporting, F_grip);

      center_of_gravity_acceleration = GIA.force_due_to_inertial_acceleration_ / mass;

      % Gravito-Inertial Acceleration (g-a) vector [m/s^2]
      GIA.gia_vector_ = gravity - center_of_gravity_acceleration;

      GIA.checkEquilibrium();

      GIA.stability_polyhedron_.setPlanePoint(GIA.max_acceleration_in_normal_direction_);
      GIA.stability_polyhedron_.setPlaneVector(GIA.normal_vector_);

      % GIA stable region
      tumbling_axes = GIA.tumbling_axes_;
      number_of_tumbling_axes = GIA.number_of_tumbling_axes_;
      unit_normal_vector = GIA.unit_normal_vector_;
      GIA.stability_polyhedron_.calcStableRegion(CoM, end_effector_position, tumbling_axes, number_of_tumbling_axes, unit_normal_vector);
    end

    function calcTumblingAxis(GIA, kNumLimb, is_grasping)
    % Obtain all possible tumbling axes numbering, accordingly to the supporting leg IDs.
    % This function requires that the numbering order of legs follows either a clockwise or counter-clockwise sequence.
    %
    % Input - kNumLimb   : Total number of limbs
    %       - is_grasping: Boolean for grasping limb ID

      tumbling_axes = [];

      % TODO: Modify this code
      if (sum(is_grasping) > 1)

        cnt = 1;
        for i = 1 : kNumLimb
          if (is_grasping(1, i))
            tumbling_axes(cnt, 1) = i;

            for j = i + 1 : kNumLimb
              if (is_grasping(1, j))
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
        number_of_tumbling_axes = cnt;

      else

        % One or none supporting legs case
        for i = 1 : kNumLimb
          if (is_grasping(1, i))
            tumbling_axes(1, :) = [i, i];
          end
        end
        number_of_tumbling_axes = 0;

      end

      GIA.tumbling_axes_ = tumbling_axes;
      GIA.number_of_tumbling_axes_ = number_of_tumbling_axes;
    end

    function calcNormalVector(GIA, p_g, end_effector_position)
    % Calculate vector normal to tumbling axes from center of gravity, which is the normal vector to the limit planes
    %
    % Input - p_g                  : Center of Gravity position [m] (3 x 1 vector)
    %       - end_effector_position: End-effector positions (= [p_1, p_2, ... p_n]) [m] (3 x n matrix)

      tumbling_axes = GIA.tumbling_axes_;
      number_of_tumbling_axes = GIA.number_of_tumbling_axes_;
      normal_vector = zeros(3, number_of_tumbling_axes);
      unit_normal_vector = zeros(3, number_of_tumbling_axes);

      for tumbling_axis_id = 1 : number_of_tumbling_axes
        % Limb IDs for tumbling axis
        limb_a = tumbling_axes(tumbling_axis_id, 1);
        limb_b = tumbling_axes(tumbling_axis_id, 2);
        % Tumbling axis initial and final points
        p_a = end_effector_position(:, limb_a);
        p_b = end_effector_position(:, limb_b);

        normal_vector(:, tumbling_axis_id) = cross(p_g - p_a, p_g - p_b);
        unit_normal_vector(:, tumbling_axis_id) = normal_vector(:, tumbling_axis_id) / norm(normal_vector(:, tumbling_axis_id));
      end

      GIA.normal_vector_ = normal_vector;
      GIA.unit_normal_vector_ = unit_normal_vector;
    end

    function calcMaximumAccelerationForNormalVectorDirection(GIA, ...
        mass, F_0, M_0, end_effector_position, is_grasping, F_hold)
    % Calculate maximum acceleration for the normal directions of tumbling axes
    %
    % Input  - mass                          : Total mass of the robot [kg] (scalar)
    %        - F_0                           : External force acting at the center of gravity [N] (3x1 vector)
    %        - M_0                           : External moment acting at the center of gravity [Nm] (3x1 vector)
    %        - end_effector_position         : End-effector positions (= [p_1, p_2, ... p_n]) [m] (3 x n matrix)
    %        - is_grasping                   : Boolean for grasping limb ID
    %        - F_hold                        : Maximum holding force [N] (scalar)

      number_of_tumbling_axes = GIA.number_of_tumbling_axes_;

      if (number_of_tumbling_axes == 0)
        GIA.max_acceleration_in_normal_direction_ = zeros(3, 1);
        return;
      end

      kNumLimb = size(end_effector_position, 2);
      tumbling_axes = GIA.tumbling_axes_;
      normal_vector = GIA.normal_vector_;
      unit_normal_vector = GIA.unit_normal_vector_;

      max_acceleration_in_normal_dir = zeros(3, number_of_tumbling_axes);

      for tumbling_axis_id = 1 : number_of_tumbling_axes
        % Limb IDs for tumbling axis
        limb_a = tumbling_axes(tumbling_axis_id, 1);
        limb_b = tumbling_axes(tumbling_axis_id, 2);
        % Tumbling axis initial and final points
        p_a = end_effector_position(:, limb_a);
        p_b = end_effector_position(:, limb_b);

        % Moment due to external force/moment
        M_ab = M_0' * (p_a - p_b) + F_0' * cross(p_b, p_a);

        % Moment due to holding force
        for limb_id = 1 : kNumLimb
          if (limb_id ~= limb_a && limb_id ~= limb_b && is_grasping(1, limb_id))
            p_j = end_effector_position(:, limb_id);
            M_ab = M_ab + F_hold * [0, 0, -1] * cross(p_b - p_j, p_a - p_j);
          end
        end

        n_ab = normal_vector(:, tumbling_axis_id);
        n_ab_u = unit_normal_vector(:, tumbling_axis_id);

        % Maximum GIA
        max_acceleration_in_normal_dir(:, tumbling_axis_id) = M_ab / (mass * norm(n_ab)) * n_ab_u;
      end

      GIA.max_acceleration_in_normal_direction_ = max_acceleration_in_normal_dir;
    end

    function checkEquilibrium(GIA)
    % Check equilibrium based on the current GIA and maximum acceleration limit
      GIA.is_equilibrium_ = true;
      number_of_tumbling_axes = GIA.number_of_tumbling_axes_;
      unit_normal_vector = GIA.unit_normal_vector_;
      max_acceleration_in_normal_dir = GIA.max_acceleration_in_normal_direction_;

      if (number_of_tumbling_axes == 0)
        GIA.is_equilibrium_ = false;
      end

      for tumbling_axis_id = 1 : number_of_tumbling_axes
        not_equilibrium_condition = GIA.gia_vector_' * unit_normal_vector(:, tumbling_axis_id) > norm(max_acceleration_in_normal_dir(:, tumbling_axis_id));
        if (not_equilibrium_condition)
          GIA.is_equilibrium_ = false;
        end
      end
    end

    function calcGIAAccelerationMargin(GIA)
    % Calculate acceleration margin based on the stability polyhedron
    % Considering the tumble stability with the addition of gripping forces to prevent the tumbling motion, the following equation describe the limit condition for one tumbling axis
    %
    %   m.a_gi.{(pg - pa) x (pg - pb)} = sum{Fj.{(pb - pj) x (pa - pj)}} + M0.(pa - pb) + F0.(pb x pa)
    %
    % The GIA acceleration margin is the following, considering all tumbling axes
    %
    %                                   a_gi.{(pg-pa)x(pg-pb)}
    %   gia_marg =  ||a_gi_lim||  -  ----------------------------
    %                                     ||(pg-pa)x(pg-pb)||
    %
    %     a_gi: Gravito-inertial acceleration
    %     pa  : Position of the first point of the tumbling axis
    %     pb  : Position of the second point of the tumbling axis
    %     pg  : Position of the center of gravity

      gia_margin_ab = zeros(1, GIA.number_of_tumbling_axes_);

      if (~GIA.is_equilibrium_)
        GIA.gia_margin_for_each_tumbling_axis_ = gia_margin_ab;
        GIA.gia_margin_ = 0.0;  % If not in equilibrium, margin is zero
        return;
      end

      plane_point = GIA.stability_polyhedron_.getPlanePoint();
      plane_vector = GIA.stability_polyhedron_.getPlaneVector();

      for axis_id = 1 : GIA.number_of_tumbling_axes_
        gia_margin_ab(1, axis_id) = norm(plane_point(:, axis_id)) - GIA.gia_vector_' * plane_vector(:, axis_id) / norm(plane_vector(:, axis_id));
      end

      GIA.gia_margin_for_each_tumbling_axis_ = gia_margin_ab;
      GIA.gia_margin_ = min(gia_margin_ab);
    end

    function calcGIAInclinationMargin(GIA)
    % Calculate inclination margin based on the stability polyhedron
    % Considering the tumble stability with the addition of gripping forces to prevent the tumbling motion, the following equation describe the limit condition for one tumbling axis
    %
    %   m.a_gi.{(pg - pa) x (pg - pb)} = sum{Fj.{(pb - pj) x (pa - pj)}} + M0.(pa - pb) + F0.(pb x pa)
    %
    % The angular margin for the total acceleration is the following, considering all tumbling axes
    %
    %                                (pg-pa)x(pg-pb).a_gi               ||(a_gi_lim||
    %   gia_inc_marg = min(acos(-------------------------------) - acos(-------------))
    %                             ||(pg-pa)x(pg-pb)|| ||a_gi||            ||a_gi||
    %
    %     a_gi: Gravito-inertial acceleration
    %     pa  : Position of the first point of the tumbling axis
    %     pb  : Position of the second point of the tumbling axis
    %     pg  : Position of the center of gravity

      gia_inclination_margin_ab = zeros(1, GIA.number_of_tumbling_axes_);

      if (~GIA.is_equilibrium_)
        GIA.gia_inclination_margin_for_each_tumbling_axis_ = gia_inclination_margin_ab;
        GIA.gia_inclination_margin_ = 0.0;
        return;
      end

      plane_point = GIA.stability_polyhedron_.getPlanePoint();
      plane_vector = GIA.stability_polyhedron_.getPlaneVector();
      gia_vector = GIA.gia_vector_;

      for axis_id = 1 : GIA.number_of_tumbling_axes_
        if (norm(gia_vector) < norm(plane_point(:, axis_id)))
          % If acceleration is smaller than limit, inclination does not affect margin
          gia_inclination_margin_ab(1, axis_id) = pi;
        else
          gia_inclination_margin_ab(1, axis_id) = ...
            acos(plane_vector(:, axis_id)' * gia_vector / (norm(plane_vector(:, axis_id)) * norm(gia_vector))) - ...
            acos(norm(plane_point(:, axis_id)) / norm(gia_vector));
        end
      end

      GIA.gia_inclination_margin_for_each_tumbling_axis_ = gia_inclination_margin_ab;
      GIA.gia_inclination_margin_ = rad2deg(min(gia_inclination_margin_ab));
    end

    function visualizeGIAVector(GIA, robot, animation)
      arguments (Input)
        GIA;
        robot     (1, 1) {mustBeA(robot,     "Robot")};
        animation (1, 1) {mustBeA(animation, "Animation")};
      end

      if (~GIA.kVisualizeGIAVector_)
        return;
      end

      CoM = robot.getStateVariable().getCoM();
      kColor = GIA.kGIAVectorColor_;
      kWidth = GIA.kGIAVectorWidth_;
      vec_magnitude = GIA.gia_vector_ * animation.getAccelerationExpansionFactor();
      animation.visualizeVector(CoM, vec_magnitude, kColor, kWidth);
    end

  end

  %% Getter
  methods (Access = public)

    function gia_vector = getGIAVector(GIA)
      gia_vector = GIA.gia_vector_;
    end

    function gravito_inertial_acceleration_margin = getGIAM(GIA)
      gravito_inertial_acceleration_margin = GIA.gia_margin_;
    end

    function inclination_margin = getGIAInclinationMargin(GIA)
      inclination_margin = GIA.gia_inclination_margin_;
    end

    function stability_polyhedron = getStabilityPolyhedron(GIA)
      stability_polyhedron = GIA.stability_polyhedron_;
    end

    function visualize_GIA_vector = getVisualizeGIAVector(GIA)
      visualize_GIA_vector = GIA.kVisualizeGIAVector_;
    end

  end

end  % GravitoInertialAcceleration
