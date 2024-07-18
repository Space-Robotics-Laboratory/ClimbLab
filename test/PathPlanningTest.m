classdef PathPlanningTest < matlab.unittest.TestCase
  % path_planning_test = PathPlanningTest;
  % result = path_planning_test.run

  methods (Test)
    function testGlobalPathPlan(path_planning_test)
      clc;

      current_position = [0.0; 0.0; 0.0];
      goal_position = [1.0; 0.0; 0.0];
      global_path_plan_type = "straight_toward_the_goal_directory";
      local_path_plan_type = "";

      path_planning = PathPlanning(global_path_plan_type, local_path_plan_type);

      path_planning = path_planning.planGlobalPath(current_position, goal_position);

      moving_direction = path_planning.global_path.getMovingDirection();
      expected_moving_direction = ...
        (goal_position - current_position) / norm(goal_position - current_position);

      path_planning_test.verifyEqual(moving_direction, expected_moving_direction);
    end
  end

end
% EOF