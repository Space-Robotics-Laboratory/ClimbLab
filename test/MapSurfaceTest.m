classdef MapSurfaceTest < matlab.unittest.TestCase
  % clc; close all; map_surface_test = MapSurfaceTest; result = map_surface_test.run

  methods (Test)
    function testMapSurface(map_surface_test)
      % Map param
      surface_type = "uneven";
      surface_inclination = [0.0; 0.0; 0.0];
      surface_grid_color = [0.9, 0.9, 0.9];
      surface_alpha = 1.0;
      % Graspable points param
      graspable_points_detection_type = "all";
      graspable_points_marker_style = "o";
      graspable_points_marker_size = 10.0;
      graspable_points_color = [0.0, 0.0, 0.3];
      graspable_points_alpha = 0.1;

      map_surface = MapSurface(surface_type, surface_inclination);
      map_surface = map_surface.initializeGraspablePoints(graspable_points_detection_type);

      figure(1); hold on; grid on;
      map_surface = map_surface.visualize(surface_grid_color, surface_alpha);
      map_surface.graspable_points.visualize(graspable_points_marker_style, ...
        graspable_points_marker_size, graspable_points_color, graspable_points_alpha);


      lighting gouraud;
      material shiny;
      lightangle(-10, 15);
      xlabel("x"); ylabel("y");
      axis equal;
      view(-20, 10);
    end
  end

end
% EOF