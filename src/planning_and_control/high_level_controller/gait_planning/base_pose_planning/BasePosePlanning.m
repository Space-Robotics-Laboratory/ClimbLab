classdef BasePosePlanning
% BasePosePlanning
% Plan the desired base pose
%
% Created     : 2020.04.10 by Warley Ribeiro
% Last updated: 2024.10.22 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    position_planning_type (1, 1) string;
    orientation_planning_type (1, 1) string;
    position_planner;
    orientation_planner;

    desired_position (3, 1) double;  % [m] at landing time of swing limb
    desired_orientation_dcm  (3, 3) double;  % at landing time of swing limb
  end

  %% Public Methods
  methods (Access = public)

    function base_pose_planning = BasePosePlanning(type)
    % BasePosePlanning() Constructor
      arguments (Input)
        type (2, 1) {mustBeA(type, "string")};
      end
      base_pose_planning.position_planning_type = type(1, 1);
      base_pose_planning.orientation_planning_type = type(2, 1);

      base_pose_planning.position_planner = base_pose_planning.setPositionPlanner();
      base_pose_planning.orientation_planner = base_pose_planning.setOrientationPlanner();
    end

    function base_pose_planning = plan(base_pose_planning, robot, path_planning, foothold_planning)
    % plan()
    %   Plan the desired robot base position and orientation at landing time of swing limb
      arguments (Input)
        base_pose_planning;
        robot             (1, 1) {mustBeA(robot, "Robot")};
        path_planning     (1, 1) {mustBeA(path_planning, "PathPlanning")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
      end

      if (base_pose_planning.position_planning_type == "do_nothing")
        base_pose_planning.desired_position = robot.des_SV.getBasePosition();
      else
        base_pose_planning.desired_position = base_pose_planning.position_planner.plan( ...
          robot, path_planning, foothold_planning);
      end

      if (base_pose_planning.orientation_planning_type == "do_nothing")
        base_pose_planning.desired_orientation_dcm = robot.des_SV.getBaseOrientationDCM();
      else
        % TODO: Implement orientation_planner.plan
      end

    end

  end

  %% Setter
  methods (Access = private)

    function position_planner = setPositionPlanner(base_pose_planning)
      switch (base_pose_planning.position_planning_type)
        case "do_nothing"
          position_planner = [];
        case "intersection_of_diagonal_lines"
          position_planner = IntersectionOfDiagonalLines();
        case "intersection_of_diagonal_line_and_moving_direction"
          position_planner = IntersectionOfDiagonalLineAndMovingDirection();
        otherwise
          error("ERROR: Failed to set base position planner.")
      end
    end

    function orientation_planner = setOrientationPlanner(base_pose_planning)
      switch (base_pose_planning.orientation_planning_type)
        case "do_nothing"
          orientation_planner = [];
        otherwise
          error("ERROR: Failed to set base orientation planner.")
      end
    end

  end

  %% Getter
  methods (Access = public)
    function desired_position = getDesiredBasePosition(base_pose_planning)
      desired_position = base_pose_planning.desired_position;
    end
    function desired_orientation_dcm = getDisiredOrientationDCM(base_pose_planning)
      desired_orientation_dcm = base_pose_planning.desired_orientation_dcm;
    end
  end

end
% EOF