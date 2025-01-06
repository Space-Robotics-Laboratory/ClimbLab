classdef config_example_demo_3
% config_example_demo_2
% Define simulation parameters for the representative simulation cases of the ClimbLab
% [ ] Demo (1) Static simulation to demonstrate the detachment of the gripper from an over-hanging wall
% [ ] Demo (2) Steep slope climbing of the mammalian typed robot
% [x] Demo (3) Perceptive walking of the half human sized quadrupedal robot
% --------------------------------------------------------------------
% This configuration file can reproduce the similar result of CLAWAR 2021 proceedings paper by K. Uno, W. Ribeiro et al.
% (Some parameters were re-tuned, thus the result slightly differs from the paper.)
% Proceedings Paper URL: https://link.springer.com/chapter/10.1007/978-3-030-86294-7_20
% --------------------------------------------------------------------
%
% Created     : 2021.03.09 by Kentaro Uno
% Last updated: 2023.05.12 by Masazumi Imai

% TODO: Implement perception (sensing camera) related
% TODO: Add foothold planning
% TODO: Add non-periodic gait planning
% TODO: Implement sim stop setting

  %% Environment Parameters
  properties (SetAccess = private, GetAccess = {?ConfigWorld, ?Configuration})
    kMaxSimulationTime_ (1, 1) double = 0.0;  % [s]  % TODO: 100.0
    KUseDynamics_ (1, 1) logical = false;
    kGravity_ (1, 1) double = 1.0;  % [G]

    kSimulationStopByReachingGoalWithGrasping_ (1, 1) logical = true;
      kThresholdForReachingGoal_ (1, 1) double = 0.1;
  end

  properties (SetAccess = private, GetAccess = {?ConfigTerrain, ?Configuration})
    surface_type (1, 1) string = "grid_3mx3m_dx100mm_thinned_40";
    inclination (3, 1) double = [0.0; 0.0; 0.0];  % [deg]

    stiffness_coefficient_for_GRF (1, 1) double = 100000.0;
    damping_coefficient_for_GRF   (1, 1) double = 100.0;
    stiffness_coefficient_for_GRM (1, 1) double = 0.1;
    damping_coefficient_for_GRM   (1, 1) double = 0.01;

    % Visualization
    surface_grid_color = "none";
    kVisualizeGraspablePoints_ (1, 1) logical = true;
      kGraspablePointsMarkerStyle_  (1, 1) string = "o";
      kGraspablePointsMarkerSize_   (1, 1) double = 20.0;
      kGraspablePointsColor_                      = [0.8, 0.8, 0.8];
      kGraspablePointsTransparency_ (1, 1) double = 1.0;
  end

  %% Robot Parameters
  properties (SetAccess = private, GetAccess = {?ConfigRobot, ?Configuration})
    robot_type (1, 1) string = "ALPHRED";

    % Initial robot base position in Ground frame
    initial_base_position (3, 1) double = [-0.8; 0.0; 0.5];  % [m]
    % Initial robot base orientation (euler) in Ground frame
    initial_base_orientation_euler (3, 1) double = [0.0; 0.0; 0.0];  % [deg]

    % Desired initial End-Effector (x, y) distance from robot base CoM in Ground frame
    desired_initial_EE_distance_xy_from_base_CoM (2, 1) double = [0.2; 0.2];  % [m]

    % "max_holding_force"
    gripper_detachment_detection_method (1, 1) string = "max_holding_force";

    % Position threshold for checking if gripper can grasp
    gripper_grasp_position_threshold (1, 1) double = 0.005;
    % Velocity threshold for checking if gripper can grasp
    gripper_grasp_velocity_threshold (1, 1) double = 0.05;

    % Visualization settings
    visualize_robot (1, 1) logical = true;
      base_upper_thickness (1, 1) double = 0.07;  % [m]
      base_lower_thickness (1, 1) double = 0.03;  % [m]
      link_radius (1, 1) double = 0.0185;  % [m]
      base_color (3, 1) double = [0.1, 0.1, 0.1];
      limb_color (3, 1) double = [0.1, 0.1, 0.1];
      base_alpha (1, 1) double = 0.8;  % [0, 1]
      limb_alpha (1, 1) double = 0.8;  % [0, 1]
  end

  %% Perception Parameters
  properties (SetAccess = private, GetAccess = {?ConfigPerception, ?Configuration})
    kUseSensingCamera_ (1, 1) logical = true;
    kSensingType_ (1, 1) string = "RealSense_D435i";

    kMountingPosition_ (3, 1) double = [0.08599 / sqrt(2.0); 0.0 ; 0.07];  % [m]
    kMountingAngle_    (3, 1) double = deg2rad([0.0; -60.0; 0.0]);  % (euler) [deg]

    kFOVMinDistance_ (1, 1) double = 0.0;

    kInitialKnownAreaShape_ (1, 1) string = "circle";
    kCircularRadiusFromBaseCoM_ (1, 1) double = 0.6;  % [m]
  end

  %% Path Planning Parameters
  properties (SetAccess = private, GetAccess = {?ConfigPathPlanning, ?Configuration})
    goal_position (3, 1) double = [1.0; 0.0; 0.0];  % [m]
    global_path_plan_type (1, 1) string = "straight_toward_the_goal_direction";
    local_path_plan_type  (1, 1) string = "LPP_based_on_next_way_point";

    % Visualization
    % kVisualizeGoalPosition_ (1, 1) logical = true;  % TODO: Implement this
  end

  %% Foothold Planning Parameters
  properties (SetAccess = private, GetAccess = {?ConfigFootholdPlanning, ?Configuration})
    % Foothold selection type
    foothold_selection_type (1, 1) string = "fixed_stride";  % TODO: max_stride_to_goal_in_reachable_area
    max_allowable_stride (1, 1) double = 0.3;  % [m]

    % Visualization
    % kVisualizeNextDesiredFootholdPosition_ (1, 1) logical = true;  % TODO: Implement this
    % kVisualizeReachableArea_ (1, 1) logical = true;  % TODO: Implement this
    %   kReachableAreaLineColor_ (1, 1) string = "m";
    %   kReachableAreaLineWidth_ (1, 1) double = 1.0;
  end

  %% Gait Planning Parameters
  properties (SetAccess = private, GetAccess = {?ConfigGaitPlanning, ?Configuration})
    gait_type (1, 1) string = "periodic_crawl";  % TODO: non_periodic_gait_for_discrete_footholds

    % Non-periodic gait settings

    step_height (1, 1) double = 0.05;  % [m]
    foot_lift_up_duration   (1, 1) double = 0.0;  % [s]
    foot_lift_down_duration (1, 1) double = 0.0;  % [s]

    % Base pose planning
    base_position_planning_type    (1, 1) string = "intersection_of_diagonal_lines";
    base_orientation_planning_type (1, 1) string = "do_nothing";
  end

  %% Trajectory Planning Parameters
  properties (SetAccess = private, GetAccess = {?ConfigTrajectoryPlanning, ?Configuration})
    base_trajectory_type (1, 1) string = "5th_order_bezier";
    limb_trajectory_type (1, 1) string = "7th_order_spline";
  end

  %% Joint Controller Parameters
  properties (SetAccess = private, GetAccess = {?ConfigJointController, ?Configuration})
    controller_type (1, 1) string = "PD_control";
    proportional_gain (1, 1) double = 1500.0;
    derivative_gain   (1, 1) double = 4.5;
  end

  %% Evaluation Parameters
  properties (SetAccess = private, GetAccess = {?ConfigEvaluation, ?Configuration})
    evaluate_manipulability (1, 1) logical = true;
    evaluate_dynamic_manipulability (1, 1) logical = true;

    visualize_supporting_leg_polygon (1, 1) logical = true;
      supporting_leg_polygon_face_color = [0.0, 136.0 / 255.0, 170.0 / 255.0];
      supporting_leg_polygon_edge_color = "none";
      supporting_leg_polygon_face_transparency (1, 1) double = 0.5;

    evaluate_tumble_stability_margin (1, 1) logical = true;
  end

  %% Animation Settings
  properties (SetAccess = private, GetAccess = {?ConfigAnimationSettings, ?Configuration})
    display_animation (1, 1) logical = true;
    save_video        (1, 1) logical = true;
    frame_rate        (1, 1) double  = 20;          % [frames/s] (positive value)
    resolution        (1, 2) double  = [1280, 720];  % [px]
    show_elapsed_time (1, 1) logical = false;

    font_name (1, 1) string = "Calibri";
    font_size (1, 1) double = 25;

    % Camera related
    x_axis_limit (1, 2) double = [-1.5, 1.5];  % [m]
    y_axis_limit (1, 2) double = [-1.5, 1.5];  % [m]
    z_axis_limit (1, 2) double = [ 0.0, 0.7];  % [m]
    camera_azimuth   (1, 1) double = -10;  % [deg]
    camera_elevation (1, 1) double =  20;  % [deg]
    camera_follow_robot (1, 1) logical = false;

    acceleration_expansion_factor (1, 1) double = 0.02;

    % kVisualizeGravitationalAccelerationVector_ (1, 1) logical = true;  % TODO: Implement this

    % kVisualizeSensingCameraFoV_ (1, 1) logical = true;  % TODO: Implement this
  end

  %% Save Settings
  properties (SetAccess = private, GetAccess = {?ConfigSaveSettings, ?Configuration})
    kSaveCsvFile_ (1, 1) logical = true;

    % Time interval for saving variables (should be larger than time-step)
    kVariableSavingTimeInterval_ (1, 1) double = 0.05;

    kSaveTumbleStabilityMargin_ (1, 1) logical = true;
  end

  %% Plot Settings
  properties (SetAccess = private, GetAccess = {?ConfigPlotSettings, ?Configuration})
    kSaveGraphs_ (1, 1) logical = true;

    kPlotJointTorque_           (1, 1) logical = true;
    kPlotTumbleStabilityMargin_ (1, 1) logical = true;
    % kPlotFootholdsHistory_      (1, 1) logical = true;  % TODO: Implement this
  end

end  % config_example_demo_3
