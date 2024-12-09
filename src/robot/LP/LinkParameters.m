classdef LinkParameters
  % Link Parameters
  %% Properties
  properties (SetAccess = private, GetAccess = public)
    BB       (1, :) double     % Link connection relationship
    S0       (1, :) double     % Links connected to base (link 0)
    SS       (:, :) double     % All link connection relationship
    SE       (1, :) double     % End link (end-effector)
    J_type   (1, :) string     % Type of joint
    c0       (3, :) double     % Position vector from base CoM to i-th joint
    cc       (3, :, :) double  % Position vector from each link CoM to i-th joint
    ce       (3, :) double     % Position vector from end link CoM to end point
    Qi       (3, :) double     % Rotational relationship of links frames
    Qe       (3, :) double     % Rotational relationship of end link frames
    m0       (1, 1) double     % Mass of the base (link 0)
    m        (1, :) double     % Mass of each link
    mass     (1, 1) double     % Total mass
    inertia0 (3, 3) double     % Moment of inertia of base
    inertia  (3, :) double     % Moment of inertia of each link
    num_q    (1, 1) uint8      % Number of links/joints

    joint_allocation_type (1, 1) string % State the type of the joint configuration
    F_grip (1, 1) double                % Max. endurable gripping force
    joint_limit (:, 2) double           % Movable limitation of joint

    num_limb (1, 1) uint8              % Total number of limbs
    joints (:, :) uint8
    num_joints_per_limb (1, :) uint8   % Number of joints per limb
  end

  %% Public Methods
  methods (Access = public)

    function LP = LinkParameters(LP_file_name)
    % LinkParameters() Constructor
      arguments (Input)
        LP_file_name (1, 1) {mustBeA(LP_file_name, "string")};
      end

      kPathToLPFile = "src" + filesep + "robot" + filesep + "LP" + filesep + LP_file_name + ".m";
      if (~isfile(kPathToLPFile))
        error("ERROR: Invalid robot type is specified. LP file dose not exist.");
      end

      LP_file = str2func(LP_file_name);
      LP_tmp = LP_file();

      LP.BB = LP_tmp.BB;
      LP.S0 = LP_tmp.S0;
      LP.SS = LP_tmp.SS;
      LP.SE = LP_tmp.SE;
      LP.J_type = LP_tmp.J_type;
      LP.c0 = LP_tmp.c0;
      LP.cc = LP_tmp.cc;
      LP.ce = LP_tmp.ce;
      LP.Qi = LP_tmp.Qi;
      LP.Qe = LP_tmp.Qe;
      LP.m0 = LP_tmp.m0;
      LP.m = LP_tmp.m;
      LP.mass = LP_tmp.mass;
      LP.inertia0 = LP_tmp.inertia0;
      LP.inertia = LP_tmp.inertia;
      LP.num_q = LP_tmp.num_q;

      LP.joint_allocation_type = LP_tmp.joint_allocation_type;
      LP.F_grip = LP_tmp.F_grip;
      LP.joint_limit = LP_tmp.joint_limit;

      LP.num_limb = sum(LP.SE, 2);
      for limb_id = 1 : LP.num_limb
        LP.joints(:, limb_id) = j_num(LP, limb_id);
        LP.num_joints_per_limb(1, limb_id) = length(LP.joints(:, limb_id));
      end
    end

    function cloned_LP = clone(original_LP)
    % clone()
    %   Return link parameters (struct) which have same values as properties of original link
    %   parameters.
      prop_name = properties(original_LP);
      for i = 1 : length(prop_name)
        cloned_LP.(prop_name{i, 1}) = original_LP.(prop_name{i, 1});
      end
    end

  end

  %% Getter
  methods (Access = public)
    function BB = getLinkConnectionRelationship(LP)
      BB = LP.BB;
    end
    function SE = getEndLink(LP)
      SE = LP.SE;
    end
    function c0 = getPositionVectorFromBaseCoMToJoint(LinkParameters)
      c0 = LinkParameters.c0;
    end
    function cc = getPositionVectorFromLinkCoMToJoint(LP)
      cc = LP.cc;
    end
    function ce = getPositionVectorFromEndLinkCoMToEndPoint(LP)
      ce = LP.ce;
    end
    function Qi = getRotationalRelationshipOfLinkFrames(LP)
      Qi = LP.Qi;
    end
    function num_q = getNumberOfJoints(LinkParameters)
      num_q = LinkParameters.num_q;
    end
    function num_limb = getNumberOfLimb(LinkParameters)
      num_limb = LinkParameters.num_limb;
    end
    function joint_allocation_type = getJointAllocationType(LinkParameters)
      joint_allocation_type = LinkParameters.joint_allocation_type;
    end
    function joints = getJoints(LinkParameters)
      joints = LinkParameters.joints;
    end
    function num_joints_per_limb = getNumberOfJointsPerLimb(LinkParameters)
      num_joints_per_limb = LinkParameters.num_joints_per_limb;
    end
    function F_grip = getMaxEndurableGrippingForce(LinkParameters)
      F_grip = LinkParameters.F_grip;
    end
  end

end
% EOF
