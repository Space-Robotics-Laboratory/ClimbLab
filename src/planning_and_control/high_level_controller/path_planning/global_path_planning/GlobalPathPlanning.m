classdef GlobalPathPlanning
% GlobalPathPlanning
% Plan the global path from the current robot base position to the goal position
%
% Created     : 2021.06.28 by Keigo Haji
% Last updated: 2024.10.22 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    type (1, 1) string;
  end
  properties (SetAccess = private, GetAccess = public)
    planner;
    goal_position (3, 1) double;  % [m]
    path Trajectory;  % way points to goal
  end

  %% Public Methods
  methods (Access = public)

    function global_path_planning = GlobalPathPlanning(config, robot, terrain)
    % GlobalPathPlanning() Constructor
      arguments
        config  (1, 1) {mustBeA(config, "ConfigPathPlanning")};
        robot   (1, 1) {mustBeA(robot, "Robot")};
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
      end
      global_path_planning.type = config.getGlobalPathPlanningType();
      global_path_planning.planner = global_path_planning.setPlanner();

      robot_base_height = robot.getBaseHeightInSurfaceFrame();
      surface_inclination = terrain.getSurfaceInclination();
      goal_pos_proj_on_surface = rpy2dc(deg2rad(surface_inclination))' * config.getGoalPosition();
      global_path_planning.goal_position = goal_pos_proj_on_surface + [0.0; 0.0; robot_base_height];

      global_path_planning.path = Trajectory();
    end

    function global_path_planning = plan(global_path_planning)
    % plan()
    %   Plan the global path from robot position to goal position
      arguments (Input)
        global_path_planning;
      end
      if (global_path_planning.type == "do_nothing")
        return;
      end

      way_points = global_path_planning.planner.plan(global_path_planning.goal_position);
      global_path_planning.path = global_path_planning.path.addPoint(way_points);
    end

  end

  %% Private Methods
  methods (Access = private)

    function planner = setPlanner(global_path_planning)
      switch (global_path_planning.type)
        case "do_nothing"
          planner = [];
        case "straight_toward_the_goal_direction"
          planner = StraightTowardGoalDirection();
        otherwise
          error("ERROR: Invalid global path planner is specified!!");
      end
    end

  end

  %% Getter
  methods (Access = public)
    function goal_position = getGoalPosition(global_path_planning)
      goal_position = global_path_planning.goal_position;
    end
    function global_path = getGlobalPath(global_path_planning)
      global_path = global_path_planning.path.getPoints();
    end
  end

end
% EOF