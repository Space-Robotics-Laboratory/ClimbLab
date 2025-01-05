classdef ContactState < handle
% Contact state of end-effector
%
% Created     : 2020.04.09 by Warley Ribeiro
% Last updated: 2024.12.12 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    in_contact_ (1, :) logical;
    position_ (3, :) double;
    orientation_dcm_ (3, :) double;
  end

  %% Public Methods
  methods (Access = public)

    function contact_state = ContactState(num_contact_points)
    % ContactState() Constructor
      arguments (Input)
        num_contact_points (1, 1) {mustBeA(num_contact_points, "uint8")};
      end

      contact_state.in_contact_ = false(1, num_contact_points);
      contact_state.position_ = zeros(3, num_contact_points);
      contact_state.orientation_dcm_ = zeros(3, 3 * num_contact_points);
    end

    function detectEECollision(contact_state, terrain, EE_positions, EE_orientations_dcm, EE_is_grasping)
    % detectEECollision()
    %   Detect a new contact (or losing an old one) between the robot end-effector and the ground
    %   surface, and save contact pose of End-Effectors.
      arguments (Input)
        contact_state;
        terrain             (1, 1) {mustBeA(terrain, "Terrain")};
        EE_positions        (3, :) {mustBeA(EE_positions, "double")};
        EE_orientations_dcm (3, :) {mustBeA(EE_orientations_dcm, "double")};
        EE_is_grasping      (1, :) {mustBeA(EE_is_grasping, "logical")};
      end

      kNumLimb = size(contact_state.in_contact_, 2);

      for limb_id = 1 : kNumLimb
        nearest_point = terrain.getNearestPointInWorldFrame(EE_positions(:, limb_id));
        norm_vector_at_nearest_point = terrain.getNormalVectorAtPoint(nearest_point);
        % Vector from nearest point to EE position
        vec_np2EE = EE_positions(:, limb_id) - nearest_point;
        theta = acos(dot(vec_np2EE, norm_vector_at_nearest_point));
        if (theta >= pi / 2)
          contact_state.in_contact_(1, limb_id) = true;
          if (EE_is_grasping(1, limb_id))
            continue;
          end
          contact_state.position_(:, limb_id) = nearest_point;
          contact_state.orientation_dcm_(:, 3*limb_id-2 : 3*limb_id) = ...
            EE_orientations_dcm(:, 3*limb_id-2 : 3*limb_id);
        else
          contact_state.in_contact_(1, limb_id) = false;
          % contact_state.position(:, limb_id) = NaN;
          % contact_state.orientation_dcm(:, 3*limb_id-2 : 3*limb_id) = NaN;
        end
      end
    end

  end

  %% Setter
  methods (Access = public)

    function setInContact(contact_state)
      contact_state.in_contact_
    end

    function setContactPose(contact_state, ...
        contact_position, contact_orientation_dcm)
      contact_state.position_ = contact_position;
      contact_state.orientation_dcm_ = contact_orientation_dcm;
    end

  end

  %% Getter
  methods (Access = public)

    function in_contact = getInContact(contact_state)
      in_contact = contact_state.in_contact_;
    end

    function contact_position = getPosition(contact_state)
      contact_position = contact_state.position_;
    end

    function contact_orientation_dcm = getOrientationDCM(contact_state)
      contact_orientation_dcm = contact_state.orientation_dcm_;
    end

  end

end  % ContactState
