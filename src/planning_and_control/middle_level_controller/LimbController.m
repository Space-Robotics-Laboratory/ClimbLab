classdef LimbController
  %% Properties
  properties (SetAccess = private, GetAccess = public)
  end

  %% Public Methods
  methods (Access = public)

    function limb_controller = LimbController()
    % LimbController() Constructor
    end

    function robot = control(limb_controller, ...
        time, robot, foothold_planning, gait_planning, motion_planning, terrain)
      arguments (Input)
        limb_controller;
        time              (1, 1) {mustBeA(time, "double")};
        robot             (1, 1) {mustBeA(robot, "Robot")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
        gait_planning     (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
        motion_planning   (1, 1) {mustBeA(motion_planning, "MotionPlanning")};
        terrain       (1, 1) {mustBeA(terrain, "Terrain")};
      end
      global d_time;
      des_SV_last = robot.des_SV.clone();
      des_SV_tmp = des_SV_last;
      LP_tmp = robot.LP.clone();

      desired_base_position = motion_planning.getDesiredBasePosition();
      desired_base_orientation_dcm = robot.des_SV.getBaseOrientationDCM();  % TODO: Get from motion planning
      desired_EE_positions = motion_planning.getDesiredEEPositions();

      desired_joint_angles = robot.kinematics.computeInverse( ...
        desired_base_position, desired_base_orientation_dcm, desired_EE_positions);

      des_SV_tmp.R0 = desired_base_position;
      des_SV_tmp.A0 = desired_base_orientation_dcm;
      des_SV_tmp.Q0 = dc2rpy(desired_base_orientation_dcm');
      des_SV_tmp.q = desired_joint_angles;
      % Update desired state variables using desired joint angle solved from IK
      des_SV_tmp.qd = (des_SV_tmp.q - des_SV_last.q) / d_time;
      des_SV_tmp.qdd = (des_SV_tmp.qd - des_SV_last.qd) / d_time;
      des_SV_tmp.v0 = (des_SV_tmp.R0 - des_SV_last.R0) / d_time;
      des_SV_tmp.vd0 = (des_SV_tmp.v0 - des_SV_last.v0) / d_time;
      des_SV_tmp.w0 = (des_SV_tmp.Q0 - des_SV_last.Q0) / d_time;
      des_SV_tmp.wd0 = (des_SV_tmp.w0 - des_SV_last.w0) / d_time;
      % Calculate links orientations, positions, velocities and accelerations
      des_SV_tmp = calc_aa(LP_tmp, des_SV_tmp);
      des_SV_tmp = calc_pos(LP_tmp, des_SV_tmp);
      des_SV_tmp = calc_vel(LP_tmp, des_SV_tmp);
      des_SV_tmp = calc_acc(LP_tmp, des_SV_tmp);

      robot = robot.overwriteDesiredStateVariables(des_SV_tmp);


      robot = robot.updateDesiredGripperState(time, foothold_planning, gait_planning, terrain);
    end

  end

  %% Private Methods
  methods (Access = private)
  end

end
% EOF