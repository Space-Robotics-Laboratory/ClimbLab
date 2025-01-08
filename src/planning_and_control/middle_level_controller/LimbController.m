classdef LimbController < handle
% Limb controller
%
% Created     : 2024.05.20 by Masazumi Imai
% Last updated: 2025.01.08 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
  end

  %% Public Methods
  methods (Access = public)

    function limb_controller = LimbController()
    % Constructor
    end

    function robot = control(limb_controller, ...
        time, d_time, robot, foothold_planning, gait_planning, trajectory_planning, terrain)
      arguments (Input)
        limb_controller;
        time                (1, 1) {mustBeA(time, "double")};
        d_time              (1, 1) {mustBeA(d_time, "double")};
        robot               (1, 1) {mustBeA(robot, "Robot")};
        foothold_planning   (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
        gait_planning       (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
        trajectory_planning (1, 1) {mustBeA(trajectory_planning, "TrajectoryPlanning")};
        terrain             (1, 1) {mustBeA(terrain, "Terrain")};
      end

      robot.inverseKinematics(d_time, trajectory_planning);

      robot.updateDesiredGripperState(time, foothold_planning, gait_planning, terrain);
    end

  end

  %% Private Methods
  methods (Access = private)
  end

end  % LimbController
