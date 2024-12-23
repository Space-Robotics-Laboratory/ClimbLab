classdef GraphPlotter < dynamicprops & handle
% Graph plotter
% NOTE: GraphPlotter takes over properties of ConfigPlotSettings. Please refer to ConfigPlotSettings.
%
% Created:      2020.07.27 by Warley Ribeiro
% Last updated: 2024.12.21 by Masazumi Imai

  %% Properties
  properties (Access = private)
    fig_number_ (1, 1) uint8 = 1;
    kSaveDataDirName_ (1, 1) string;
  end
  properties (SetAccess = private, GetAccess = public)
  end

  %% Public Methods
  methods (Access = public)

    function graph_plotter = GraphPlotter(config_plot_settings, run_cod, run_id)
    % GraphPlotter() Constructor
      arguments (Input)
        config_plot_settings (1, 1) {mustBeA(config_plot_settings, "ConfigPlotSettings")};
        run_cod (1, 1) {mustBeA(run_cod, "string")};
        run_id (1, 1) {mustBeA(run_id, "string")};
      end

      % Add properties from ConfigPlotSettings
      kConfigPropName = properties(config_plot_settings);
      for i = 1 : length(kConfigPropName)
        dynamic_property = addprop(graph_plotter, kConfigPropName{i, 1});
        dynamic_property.Access = "private";
        graph_plotter.(kConfigPropName{i, 1}) = config_plot_settings.(kConfigPropName{i, 1});
      end

      graph_plotter.kSaveDataDirName_ = "dat/" + run_cod + "/" + run_id;
      if (graph_plotter.kSaveGraphs_ && ~isfolder(graph_plotter.kSaveDataDirName_))
        mkdir(graph_plotter.kSaveDataDirName_);
      end
    end

    function plot(graph_plotter, robot, data_logger)
    % plot()
    %   Plot graphs
      arguments (Input)
        graph_plotter;
        robot (1, 1) {mustBeA(robot, "Robot")};
        data_logger (1, 1) {mustBeA(data_logger, "DataLogger")};
      end

      kNumLimb = robot.getLinkParameter().getNumberOfLimb();
      kNumJointsPerLimb = robot.getLinkParameter().getNumberOfJointsPerLimb();
      data = data_logger.getVariablesLog();

      if (graph_plotter.kPlotBasePosition_)
        fig_title = "Base Position";
        y_label = "\it{\bf{x}}_{\rm{b}} \rm{[m]}";
        graph_plotter.plotTimeHistoryGraph(data.time, data.base_position, fig_title, y_label);
      end

      if (graph_plotter.kPlotJointTorque_)
        for limb_id = 1 : kNumLimb
          fig_title = "Joints Torque of Limb " + num2str(limb_id);
          y_label = "\rm{Joint Torque [Nm]}";
          torque = data.joint_torque(:, kNumJointsPerLimb(1, limb_id) * (limb_id - 1) + 1 : kNumJointsPerLimb(1, limb_id) * limb_id);
          graph_plotter.plotTimeHistoryGraph(data.time, torque, fig_title, y_label);
        end
      end

      if (graph_plotter.kPlotManipulability_)
        fig_title = "Manipulability Measure";
        y_label = "\rm{Manipulability Measure [-]}";
        graph_plotter.plotTimeHistoryGraph(data.time, data.manipulability_measure, fig_title, y_label);
      end

      if (graph_plotter.kPlotDynamicManipulability_)
        fig_title = "Dynamic Manipulability Measure";
        y_label = "\rm{Dynamic Manipulability Measure [-]}";
        graph_plotter.plotTimeHistoryGraph(data.time, data.dynamic_manipulability_measure, fig_title, y_label);
      end

      if (graph_plotter.kPlotTumbleStabilityMargin_)
        fig_title = "Tumble Stability Margin";
        y_label = "\rm{Tumble Stability Margin [m]}";
        graph_plotter.plotTimeHistoryGraph(data.time, data.TSM, fig_title, y_label);
      end
    end

  end

  %% Private Methods
  methods (Access = private)

    function plotTimeHistoryGraph(graph_plotter, time_data, y_data, fig_title, y_label)
      graph_plotter.fig_number_ = graph_plotter.fig_number_ + 1;

      figure(double(graph_plotter.fig_number_));
      plot(time_data, y_data, "-", LineWidth = graph_plotter.kLineWidth_);

      title(fig_title, FontName = graph_plotter.kFontName_, FontSize = graph_plotter.kFontSize_);
      xlabel("\rm{Time [s]}", FontName = graph_plotter.kFontName_, FontSize = graph_plotter.kFontSize_);
      ylabel(y_label, FontName = graph_plotter.kFontName_, FontSize = graph_plotter.kFontSize_);
      set(gca, FontName = graph_plotter.kFontName_, FontSize = graph_plotter.kFontSize_, LineWidth = graph_plotter.kLineWidth_ / 2.0);
      set(gcf, Color = "w");
      grid on;

      if (graph_plotter.kSaveGraphs_)
        saveas(gcf, graph_plotter.kSaveDataDirName_ + "/" + fig_title + ".fig", "fig");
      end
    end

  end

end  % GraphPlotter
