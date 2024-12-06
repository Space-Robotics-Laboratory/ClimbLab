classdef Environment < ConfigEnvironmentParam & handle

  properties (SetAccess = private, GetAccess = public)
    surface_point_cloud_data  % struct
    grid_x
    grid_y
    MAP_vec  % struct
    graphicas_obj_light
  end

  methods
    % Constructor
    function Environment = Environment()
      global d_time Gravity Ez;
      d_time = Environment.time_step;
      Gravity = rpy2dc(deg2rad(Environment.surface_inclination))' * ...
        Environment.gravity * [0.0 0.0 -9.81]';
      Ez = [0 0 1]';
    end

    function initializeSurface(Environment)
      switch Environment.surface_type
        case "flat_HR"
          load("map_flat_HR.mat", "x", "y", "z");
        case "rough"
          load("map_uneven.mat", "x", "y", "z");
        otherwise
          error("Invalid surface type is specified" + newline ...
            + "Check ""surface_type"" defined in config file.");
      end
      Environment.surface_point_cloud_data.x = x;
      Environment.surface_point_cloud_data.y = y;
      Environment.surface_point_cloud_data.z = z;

      if Environment.animation_display_on && Environment.surface_show
        [X, Y] = meshgrid(x, y);
        Environment.grid_x = X;
        Environment.grid_y = Y;
        Environment.MAP_vec(1, :) = reshape(X, 1, numel(X));
        Environment.MAP_vec(2, :) = reshape(Y, 1, numel(Y));
        Environment.MAP_vec(3, :) = reshape(z, 1, numel(z));
      end
    end


    function updateAnimationSettings(Environment, time)
      if time == 0
        set(gca, "FontName", Environment.font_name, "FontSize", Environment.font_size, ...
          "LineWidth", 2);
        xlabel("\it{x} \rm{[m]}");
        ylabel("\it{y} \rm{[m]}");
        zlabel("\it{z} \rm{[m]}");

        % Define background color and window size (resolution)
        set(gcf, "Color", "w", "Position", [1, 1, Environment.animation_resolution]);
        % Equal scales for each axis
        axis equal;
        % Limits for axis
        xlim(Environment.x_axis_limit);
        ylim(Environment.y_axis_limit);
        zlim(Environment.z_axis_limit);
        grid on;
        view(Environment.camera_azimuth, Environment.camera_elevation);
      end

      if Environment.animation_elapsed_time_show
        xlabel({'\it{x} \rm{[m]}'; ['Time: ', num2str(time, '%.2f'), ' [s]']}, ...
          "FontName", Environment.font_name, "FontSize", Environment.font_size);
      end

      lighting gouraud;
      material shiny;
      Environment.graphicas_obj_light = lightangle(-10, 15);
    end

    function deleteGraphicsObjects(Environment, time, Robot)
      if time == 0
        return;
      end
      delete(Robot.graphics_obj_robot_base);
      delete(Robot.graphics_obj_robot_limb_links);
      delete(Robot.graphics_obj_robot_gripper);
      delete(Environment.graphicas_obj_light);
    end

    function visualizeSurface(Environment, time)
      if ~Environment.surface_show || time ~= 0
        return;
      end
      x = Environment.surface_point_cloud_data.x;
      y = Environment.surface_point_cloud_data.y;
      z = Environment.surface_point_cloud_data.z;
      if all(Environment.surface_inclination == 0.0)
        X = Environment.grid_x;
        Y = Environment.grid_y;
        Z = z;
      else
        MAP = rpy2dc(deg2rad(Environment.surface_inclination)) * Environment.MAP_vec;
        X = reshape(MAP(1, :), length(x), length(x));
        Y = reshape(MAP(2, :), length(y), length(y));
        Z = reshape(MAP(3, :), size(z, 1), size(z, 2));
      end

      mesh(X, Y, Z, ...
        EdgeColor = Environment.surface_grid_color, ...
        EdgeAlpha = Environment.surface_grid_alpha, ...
        FaceColor = Environment.surface_face_color, ...
        FaceAlpha = Environment.surface_face_alpha);
      colormap(Environment.surface_color);
    end
  end
end
% EOF