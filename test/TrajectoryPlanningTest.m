classdef TrajectoryPlanningTest < matlab.unittest.TestCase
  % trajectory_planning_test = TrajectoryPlanningTest; result = trajectory_planning_test.run
  % result = path_planning_test.run

  methods (Test)
    function testBezier(trajectory_planning_test)
      num_limb = uint8(4);
      base_trajectory_type = "5th_order_bezier";
      limb_trajectory_type = "7th_order_bezier";
      line_style = "--";
      color = [0.5, 0.0, 0.0];
      width = 3;

      swing_duration = 1.0;  % get from gait_planning
      step_height = 0.5;  % get from gait_planning
      current_EE_position = zeros(3, num_limb);  % get from foothold_planning
      desired_EE_position = zeros(3, num_limb);  % get from foothold_planning
      for limb_id = 1:num_limb
        if limb_id == 1
          current_EE_position(:, limb_id) = [0.0; 0.0; 0.0];
          desired_EE_position(:, limb_id) = [1.0; 0.0; 0.0];
        elseif limb_id == 2
          current_EE_position(:, limb_id) = [-2.0; 0.0; 0.0];
          desired_EE_position(:, limb_id) = [-1.0; 0.0; 0.0];
        elseif limb_id == 3
          current_EE_position(:, limb_id) = [-2.0; -2.0; 0.0];
          desired_EE_position(:, limb_id) = [-1.0; -2.0; 0.0];
        elseif limb_id == 4
          current_EE_position(:, limb_id) = [0.0; -2.0; 0.0];
          desired_EE_position(:, limb_id) = [1.0; -2.0; 0.0];
        end
      end

      figure(1); hold on;

      trajectory_planning = TrajectoryPlanning(base_trajectory_type, limb_trajectory_type, num_limb);
      trajectory_planning = ...
        trajectory_planning.initializeTrajectories(current_EE_position, line_style, color, width);

      for time = 0.0 : 0.001 : 1.0
        if time == 0.0
          trajectory_planning = trajectory_planning.planTrajectories( ...
              swing_duration, current_EE_position, desired_EE_position, step_height);
        end

        trajectory_planning = trajectory_planning.updateForCurrentTimeStep( ...
          time, swing_duration);
      end

      for limb_id = 1:num_limb
        trajectory_planning.limb_trajectory(limb_id, 1).position.planned_trajectory.visualize();
      end
      axis equal; grid on;
      xlabel("x"); ylabel("y");
      view(-20, 10);
      hold off;
    end

    function testSpline(trajectory_planning_test)
      num_limb = uint8(4);
      base_trajectory_type = "5th_order_bezier";
      limb_trajectory_type = "7th_order_spline";
      line_style = "--";
      color = [0.0, 0.5, 0.0];
      width = 3;

      swing_duration = 1.0;  % get from gait_planning
      step_height = 0.5;  % get from gait_planning
      current_EE_position = zeros(3, num_limb);  % get from foothold_planning
      desired_EE_position = zeros(3, num_limb);  % get from foothold_planning
      for limb_id = 1:num_limb
        if limb_id == 1
          current_EE_position(:, limb_id) = [0.0; 0.0; 0.0];
          desired_EE_position(:, limb_id) = [1.0; 0.0; 0.0];
        elseif limb_id == 2
          current_EE_position(:, limb_id) = [-2.0; 0.0; 0.0];
          desired_EE_position(:, limb_id) = [-1.0; 0.0; 0.0];
        elseif limb_id == 3
          current_EE_position(:, limb_id) = [-2.0; -2.0; 0.0];
          desired_EE_position(:, limb_id) = [-1.0; -2.0; 0.0];
        elseif limb_id == 4
          current_EE_position(:, limb_id) = [0.0; -2.0; 0.0];
          desired_EE_position(:, limb_id) = [1.0; -2.0; 0.0];
        end
      end

      figure(2); hold on;

      trajectory_planning = TrajectoryPlanning(base_trajectory_type, limb_trajectory_type, num_limb);
      trajectory_planning = ...
        trajectory_planning.initializeTrajectories(current_EE_position, line_style, color, width);

      for time = 0.0 : 0.001 : 1.0
        if time == 0.0
          trajectory_planning = trajectory_planning.planTrajectories( ...
              swing_duration, current_EE_position, desired_EE_position, step_height);
        end

        trajectory_planning = trajectory_planning.updateForCurrentTimeStep( ...
          time, swing_duration);
      end

      for limb_id = 1:num_limb
        trajectory_planning.limb_trajectory(limb_id, 1).position.planned_trajectory.visualize();
      end
      axis equal; grid on;
      xlabel("x"); ylabel("y");
      view(-20, 10);
    end

  end

end
% EOF
