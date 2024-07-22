classdef FootholdPlanningTest < matlab.unittest.TestCase
  % clc; clear; close all; foothold_planning_test = FootholdPlanningTest; result = foothold_planning_test.run

  methods (Test)
    function testFootholdPlanning(foothold_planning_test)
      num_limb = uint8(4);
      % Foothold planning param
      foothold_selection_type = "fixed_stride";
      sequence = [2, 1, 3, 4];
      step_length = 0.05;
      current_EE_position(1:3, 1) = [0.0; 0.0; 0.0];
      current_EE_position(1:3, 2) = [-0.2; 0.0; 0.0];
      current_EE_position(1:3, 3) = [-0.2; -0.2; 0.0];
      current_EE_position(1:3, 4) = [0.0; -0.2; 0.0];
      % Map param
      surface_type = "uneven";
      surface_inclination = [0.0; 0.0; 0.0];
      % Graspable points param
      graspable_points_detection_type = "all";

      map_surface = MapSurface(surface_type, surface_inclination);
      map_surface = map_surface.initializeGraspablePoints(graspable_points_detection_type);


      foothold_planning = FootholdPlanning(foothold_selection_type, num_limb);
      foothold_planning = foothold_planning.initialize(sequence, step_length, current_EE_position);

      current_time = 0.0;
      moving_direction = [1.0; 0.0; 0.0];
      foothold_planning = foothold_planning.plan(current_time, moving_direction, map_surface.getGraspablePoints);
    end
  end

end
% EOF