classdef Animation < dynamicprops & handle
  %% Properties
  properties (SetAccess = private, GetAccess = public)
    fig (1, 1) % matlab.ui.Figure
    sim_video VideoWriter;
    graphics_obj_light (1, 1) matlab.graphics.primitive.Light
    graphics_obj_vector matlab.graphics.primitive.Patch;
  end
  properties (Access = private)
    kSaveDataDirName_ (1, 1) string;
  end

  %% Public methods
  methods (Access = public)

    function animation = Animation(config_animation_settings, run_cod, run_id)
    % Animation() Constructor
    %
    % Input : run_cod  - Program identification code
    %         run_id   - Run identification
      arguments (Input)
        config_animation_settings ...
          (1, 1) {mustBeA(config_animation_settings, "ConfigAnimationSettings")};
        run_cod (1, 1) {mustBeA(run_cod, "string")};
        run_id  (1, 1) {mustBeA(run_id,  "string")};
      end

      animation.fig = figure(Visible = "off");
      set(groot, "CurrentFigure", animation.fig);
      hold on;

      % Add properties from config
      customized_config_prop_name = properties(config_animation_settings);
      for i = 1:length(customized_config_prop_name)
        dynamic_property = addprop(animation, customized_config_prop_name{i, 1});
        dynamic_property.Access = "private";
        animation.(customized_config_prop_name{i, 1}) = ...
          config_animation_settings.(customized_config_prop_name{i, 1});
      end

      % Fonts
      set(gca, "FontName", animation.font_name, ...
        "FontSize", animation.font_size, "LineWidth", 2);
      % Labels
      xlabel("\it{x} \rm{[m]}");
      ylabel("\it{y} \rm{[m]}");
      zlabel("\it{z} \rm{[m]}");
      % Background color and window size (resolution)
      set(gcf, "Color", "w", "Position", [1, 1, animation.resolution]);
      % Axis
      axis equal;
      xlim(animation.x_axis_limit);
      ylim(animation.y_axis_limit);
      zlim(animation.z_axis_limit);
      grid on;
      % Camera angle
      view(animation.camera_azimuth, animation.camera_elevation);

      animation.kSaveDataDirName_ = "dat" + filesep + run_cod + filesep + run_id;
      if (animation.save_video && ~isfolder(animation.kSaveDataDirName_))
        mkdir(animation.kSaveDataDirName_);
      end

    end

    function setLight(animation)
    % setLight()
    %   Set light object
      lighting gouraud;
      material shiny;
      animation.graphics_obj_light = lightangle(-10, 15);
    end

    function visualizeVector(animation, origin, vec_magnitude, vec_color, vec_width)
      arguments
        animation;
        origin (3, 1) {mustBeA(origin, "double")};
        vec_magnitude (3, 1) {mustBeA(vec_magnitude, "double")};
        vec_color;
        vec_width (1, 1) {mustBeA(vec_width, "double")};
      end
      destination = origin + vec_magnitude;
      vector = [origin, destination];
      vec_arrowhead_width = vec_width * 2;

      if (isempty(animation.graphics_obj_vector))
        idx = 1;
      else
        idx = length(animation.graphics_obj_vector) + 1;
      end
      arrow = visualizeArrow(vector(:, 1), vector(:, 2), ...
        "FaceColor", vec_color, "ArrowLineWidth", vec_width, ...
        "ArrowHeadWidth", vec_arrowhead_width, "FaceLighting", 'none', "AmbientStrength", 1.0);
      animation.graphics_obj_vector(idx, 1) = arrow;
    end

    function resetGraphicsObjects(animation, time, robot)
    % resetGraphicsObjects()
    %   Delete graphics objects from animation figure
      arguments (Input)
        animation;
        time  (1, 1) {mustBeA(time, "double")};
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      if (time == 0.0)
        return;
      end
      robot.graphics_.deleteGripperGraphics();
      delete(animation.graphics_obj_light);
      delete(animation.graphics_obj_vector);
    end

    function createVideoFile(animation, run_id)
    % createVideoFile()
    %   Create file to save simulation video and define video parameters, such as quality and frame
    %   rate.
    %
    % Input : run_id   - Run identification
      arguments (Input)
        animation;
        run_id (1, 1) {mustBeA(run_id, "string")};
      end

      if (~animation.save_video)
        return;
      end

      dir_name = animation.kSaveDataDirName_;
      animation.sim_video = VideoWriter(dir_name + filesep + run_id + "_video" + animation.video_file_extension);
      animation.sim_video.Quality = 100;
      animation.sim_video.FrameRate = animation.frame_rate;
      open(animation.sim_video);
    end

    function writeVideoFile(animation)
      drawnow limitrate nocallbacks;

      if (animation.save_video)
        writeVideo(animation.sim_video, getframe(animation.fig));
      end
    end

    function saveVideoFile(animation)
      if (animation.save_video)
        close(animation.sim_video);
      end
    end

  end

  %% Getter
  methods (Access = public)
    function fig = getFigure(animation)
      fig = animation.fig;
    end

    function frame_rate = getFrameRate(animation)
      frame_rate = animation.frame_rate;
    end

    function acceleration_expansion_factor = getAccelerationExpansionFactor(animation)
      acceleration_expansion_factor = animation.acceleration_expansion_factor;
    end

    function force_expansion_factor = getForceExpansionFactor(animation)
      force_expansion_factor = animation.force_expansion_factor;
    end
    function GRF_vec_show = getGroundReactionForceVectorShow(animation)
      GRF_vec_show = animation.ground_reaction_force_vec_show;
    end
    function [vec_color, vec_width] = getGroundReactionForceVisSettings(animation)
      vec_color = animation.ground_reaction_force_vec_color;
      vec_width = animation.ground_reaction_force_vec_width;
    end
  end

end
% EOF
