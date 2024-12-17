classdef config_example_demo_2
% config_example_demo_2
% Define simulation parameters for the representative simulation cases of the ClimbLab
% [ ] Demo (1) Static simulation to demonstrate the detachment of the gripper from an over-hanging wall
% [x] Demo (2) Steep slope climbing of the mammalian typed robot
% [ ] Demo (3) Perceptive walking of the half human sized quadrupedal robot
% --------------------------------------------------------------------
% This configuration file can reproduce the similar result of CLAWAR 2021 proceedings paper by K. Uno, W. Ribeiro et al.
% (Some parameters were re-tuned, thus the result slightly differs from the paper.)
% Proceedings Paper URL: https://link.springer.com/chapter/10.1007/978-3-030-86294-7_20
% --------------------------------------------------------------------
%
% Created     : 2021.03.02 by Kentaro Uno
% Last updated: 2021.09.19 by Kentaro Uno

% TODO: Add equilibrium settings (tsm and gia)
% TODO: Add animation of support triangle, gia stable region, gia vector
% TODO: Add save settings (tsm, gia, manipulability, dynamic manipulability, joint max torque, joint rms torque, cot)
% TODO: Add plot settings (tsm, gia, manipulability, dynamic manipulability, joint max torque, joint rms torque, cot)
% TODO: Need to check if sensing camera and matching settings are necessary

  %% Environment Parameters
  properties (SetAccess = private, GetAccess = {?ConfigWorld, ?Configuration})
    max_simulation_time (1, 1) double = 16.0;  % [s]
    use_dynamics (1, 1) logical = true;
    gravity (1, 1) double = 1.0;  % [G]
  end

  properties (SetAccess = private, GetAccess = {?ConfigTerrain, ?Configuration})
    surface_type (1, 1) string = "flat_HR_5mx5m";
    inclination (3, 1) double = [0.0; -45.0; 0.0];  % [deg]

    stiffness_coefficient_for_GRF (1, 1) double = 100000.0;
    damping_coefficient_for_GRF (1, 1) double = 100.0;
    stiffness_coefficient_for_GRM (1, 1) double = 0.1;
    damping_coefficient_for_GRM (1, 1) double = 0.01;

    % Visualization
    surface_grid_color = "white";
  end

  %% Robot Parameters
  properties (SetAccess = private, GetAccess = {?ConfigRobot, ?Configuration})
    robot_type (1, 1) string = "ANYmal_B";

    % Initial robot base position in Ground frame
    initial_base_position (3, 1) double = [0.0; 0.0; 0.25];  % [m]
    % Initial robot base orientation (euler) in Ground frame
    initial_base_orientation_euler (3, 1) double = [0.0; 0.0; 0.0];  % [deg]

    % Desired initial End-Effector (x, y) distance from robot base CoM in Ground frame
    desired_initial_EE_distance_xy_from_base_CoM (2, 1) double = [0.35; 0.35];  % [m]

    % "max_holding_force"
    gripper_detachment_detection_method (1, 1) string = "max_holding_force";

    % Visualization settings
    visualize_robot (1, 1) logical = true;
      base_upper_thickness (1, 1) double = 0.20;  % [m]
      base_lower_thickness (1, 1) double = 0.05;  % [m]
      link_radius (1, 1) double = 0.03;  % [m]
      base_color (3, 1) double = [0.0, 0.3, 0.6];
      limb_color (3, 1) double = [0.3, 0.3, 0.3];
      base_alpha (1, 1) double = 1.0;  % [0, 1]
      limb_alpha (1, 1) double = 1.0;  % [0, 1]
  end

  %% Path Planning Parameters
  properties (SetAccess = private, GetAccess = {?ConfigPathPlanning, ?Configuration})
    goal_position (3, 1) double = [0.0; 0.0; 0.0];  % [m]
    % Global Path Planning method
    global_path_plan_type (1, 1) string = "straight_toward_the_goal_direction";
    % Local Path Planning method
    local_path_plan_type  (1, 1) string = "LPP_based_on_next_way_point";
  end

  %% Foothold Planning Parameters
  properties (SetAccess = private, GetAccess = {?ConfigFootholdPlanning, ?Configuration})
    % Foothold selection type
    foothold_selection_type (1, 1) string = "fixed_stride";
    max_allowable_stride (1, 1) double = 0.1;  % [m]
  end

  %% Gait Planning Parameters
  properties (SetAccess = private, GetAccess = {?ConfigGaitPlanning, ?Configuration})
    gait_type (1, 1) string = "periodic_crawl";

    % Periodic gait settings
    gait_period (1, 1) double = 8.0;  % [s]
    duty_factor (1, 1) double = 0.75;  % [0, 1]
    % Gait sequence
    % 1st dim: Limb number(s) starting at same timing during gait cycle
    % 2nd dim: Limb number(s) starting at different timing during gait cycle
    sequence uint8 = [1, 3, 4, 2];

    step_height (1, 1) double = 0.1;  % [m]
    foot_lift_up_duration   (1, 1) double = 0.0;  % [s]
    foot_lift_down_duration (1, 1) double = 0.0;  % [s]

    % Base pose planning
    base_position_planning_type    (1, 1) string = "intersection_of_diagonal_lines";
    base_orientation_planning_type (1, 1) string = "do_nothing";
  end

  %% Trajectory Planning Parameters
  properties (SetAccess = private, GetAccess = {?ConfigTrajectoryPlanning, ?Configuration})
    % Base CoM trajectory type
    base_trajectory_type (1, 1) string = "5th_order_bezier";
    % Limb end-effector trajectory type
    limb_trajectory_type (1, 1) string = "7th_order_bezier";

    visualize_limb_trajectory (1, 1) logical = false;
  end

  %% Joint Controller Parameters
  properties (SetAccess = private, GetAccess = {?ConfigJointController, ?Configuration})
    controller_type (1, 1) string = "PD_control";
    proportional_gain (1, 1) double = 850.0;
    derivative_gain   (1, 1) double = 3.0;
  end

  %% Evaluation Parameters
  properties (SetAccess = private, GetAccess = {?ConfigEvaluation, ?Configuration})
    evaluate_manipulability (1, 1) logical = true;
    evaluate_dynamic_manipulability (1, 1) logical = true;
  end

  %% Animation Settings
  properties (SetAccess = private, GetAccess = {?ConfigAnimationSettings, ?Configuration})
    display_animation (1, 1) logical = true;
    frame_rate        (1, 1) double  = 20;          % [frames/s] (positive value)
    resolution        (1, 2) double  = [640, 480];  % [px]
    show_elapsed_time (1, 1) logical = false;

    font_name (1, 1) string = "Calibri";
    font_size (1, 1) double = 20;

    % Camera related
    x_axis_limit (1, 2) double = [-0.5, 1.0];  % [m]
    y_axis_limit (1, 2) double = [-1.0, 1.0];  % [m]
    z_axis_limit (1, 2) double = [-0.5, 1.0];  % [m]
    camera_azimuth   (1, 1) double = -25;  % [deg]
    camera_elevation (1, 1) double =  10;  % [deg]
    camera_follow_robot (1, 1) logical = false;

    ground_reaction_force_vec_show (1, 1) logical = false;
  end

  %% Save Settings
  properties (SetAccess = private, GetAccess = {?ConfigSaveSettings, ?Configuration})
    % Time interval for saving variables (should be larger than time-step)
    variable_saving_time_interval (1, 1) double = 0.05;
    % Save basic variables to csv file
    save_csv_file (1, 1) logical = true;
  end

end  % config_example_demo_2
