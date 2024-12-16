classdef BasePosePlanning < handle
% BasePosePlanning
% Plan the desired base pose
%
% Created     : 2020.04.10 by Warley Ribeiro
% Last updated: 2024.12.12 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    kPositionPlanningType_    (1, 1) string;
    kOrientationPlanningType_ (1, 1) string;
    position_planner_;
    orientation_planner_;

    desired_position_        (3, 1) double;  % [m] at landing time of swing limb
    desired_orientation_dcm_ (3, 3) double;  % at landing time of swing limb
  end

  %% Methods called only from GaitPlanning
  methods (Access = ?GaitPlanning)

    function base_pose_planning = BasePosePlanning(type)
    % BasePosePlanning() Constructor
      arguments (Input)
        type (2, 1) {mustBeA(type, "string")};
      end

      base_pose_planning.kPositionPlanningType_ = type(1, 1);
      base_pose_planning.kOrientationPlanningType_ = type(2, 1);

      base_pose_planning.setPositionPlanner();
      base_pose_planning.setOrientationPlanner();
    end

    function plan(base_pose_planning, robot, path_planning, foothold_planning)
    % plan()
    %   Plan the desired robot base position and orientation at landing time of swing limb
      arguments (Input)
        base_pose_planning;
        robot             (1, 1) {mustBeA(robot, "Robot")};
        path_planning     (1, 1) {mustBeA(path_planning, "PathPlanning")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
      end

      if (base_pose_planning.kPositionPlanningType_ == "do_nothing")
        base_pose_planning.desired_position_ = robot.des_SV.getBasePosition();
      else
        base_pose_planning.desired_position_ = base_pose_planning.position_planner_.plan( ...
          robot, path_planning, foothold_planning);
      end

      if (base_pose_planning.kOrientationPlanningType_ == "do_nothing")
        base_pose_planning.desired_orientation_dcm_ = robot.des_SV_.getBaseOrientationDCM();
      else
        % TODO: Implement orientation_planner.plan
      end

    end

  end

  %% Setter
  methods (Access = private)

    function setPositionPlanner(base_pose_planning)
      switch (base_pose_planning.kPositionPlanningType_)
        case "do_nothing"
          position_planner = [];
        case "intersection_of_diagonal_lines"
          position_planner = IntersectionOfDiagonalLines();
        case "intersection_of_diagonal_line_and_moving_direction"
          position_planner = IntersectionOfDiagonalLineAndMovingDirection();
        otherwise
          error("ERROR: Failed to set base position planner.")
      end
      base_pose_planning.position_planner_ = position_planner;
    end

    function setOrientationPlanner(base_pose_planning)
      switch (base_pose_planning.kOrientationPlanningType_)
        case "do_nothing"
          orientation_planner = [];
        otherwise
          error("ERROR: Failed to set base orientation planner.")
      end
      base_pose_planning.orientation_planner_ = orientation_planner;
    end

  end

  %% Getter
  methods (Access = public)
    function desired_position = getDesiredBasePosition(base_pose_planning)
      desired_position = base_pose_planning.desired_position_;
    end
    function desired_orientation_dcm = getDesiredOrientationDCM(base_pose_planning)
      desired_orientation_dcm = base_pose_planning.desired_orientation_dcm_;
    end
  end

end  % BasePosePlanning
