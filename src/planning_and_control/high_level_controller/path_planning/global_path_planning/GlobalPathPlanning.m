classdef GlobalPathPlanning < handle
% GlobalPathPlanning
% Plan the global path from the current robot base position to the goal position
%
% Created     : 2021.06.28 by Keigo Haji
% Last updated: 2024.12.07 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    kType_ (1, 1) string;
  end
  properties (SetAccess = private, GetAccess = public)
    planner_;
    kGoalPosition_ (3, 1) double;  % [m]
    path_ TrajectoryHistory;  % way points to goal
  end

  %% Public Methods
  methods (Access = public)

    function global_path_planning = GlobalPathPlanning(config_path_planning, robot, terrain)
    % GlobalPathPlanning() Constructor
      arguments
        config_path_planning (1, 1) {mustBeA(config_path_planning, "ConfigPathPlanning")};
        robot                (1, 1) {mustBeA(robot,                "Robot")};
        terrain              (1, 1) {mustBeA(terrain,              "Terrain")};
      end

      global_path_planning.kType_ = config_path_planning.getGlobalPathPlanningType();
      global_path_planning.planner_ = global_path_planning.setPlanner();

      robot_base_height = robot.getBaseHeightInSurfaceFrame();
      kSurfaceInclination = terrain.getSurfaceInclination();
      goal_pos_proj_on_surface = rpy2dc(deg2rad(kSurfaceInclination))' * config_path_planning.getGoalPosition();
      global_path_planning.kGoalPosition_ = goal_pos_proj_on_surface + [0.0; 0.0; robot_base_height];

      global_path_planning.path_ = TrajectoryHistory();
    end

    function plan(global_path_planning)
    % plan()
    %   Plan the global path from robot position to goal position
      arguments (Input)
        global_path_planning;
      end

      if (global_path_planning.kType_ == "do_nothing")
        return;
      end

      way_points = global_path_planning.planner_.plan(global_path_planning.kGoalPosition_);
      global_path_planning.path_.addPoint(way_points);
    end

  end

  %% Setter
  methods (Access = private)

    function planner = setPlanner(global_path_planning)
      switch (global_path_planning.kType_)
        case "do_nothing"
          planner = [];
        case "straight_toward_the_goal_direction"
          planner = StraightTowardGoalDirection();
        otherwise
          error("ERROR: Invalid global path planner is specified!");
      end
    end

  end

  %% Getter
  methods (Access = public)
    function goal_position = getGoalPosition(global_path_planning)
      goal_position = global_path_planning.kGoalPosition_;
    end
    function global_path = getGlobalPath(global_path_planning)
      global_path = global_path_planning.path_.getPoints();
    end
  end

end  % GlobalPathPlanning
