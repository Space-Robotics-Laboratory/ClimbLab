classdef config_example_demo_1
% config_example_demo_1
% Define simulation parameters for the representative simulation cases of the ClimbLab
% [x] Demo (1) Static simulation to demonstrate the detachment of the gripper from an over-hanging wall
% [ ] Demo (2) Steep slope climbing of the mammalian typed robot
% [ ] Demo (3) Perceptive walking of the half human sized quadrupedal robot
% --------------------------------------------------------------------
% This configuration file can reproduce the similar result of CLAWAR 2021 proceedings paper by K. Uno, W. Ribeiro et al.
% (Some parameters were re-tuned, thus the result slightly differs from the paper.)
% Proceedings Paper URL: https://link.springer.com/chapter/10.1007/978-3-030-86294-7_20
% --------------------------------------------------------------------
%
% Created     : 2021.03.02 by Kentaro Uno
% Last updated: 2021.09.19 by Kentaro Uno

  %% Environment Parameters
  properties (SetAccess = private, GetAccess = {?ConfigWorld, ?Configuration})
    max_simulation_time (1, 1) double = 1.0;  % [s]
    use_dynamics (1, 1) logical = true;
    gravity (1, 1) double = 1.0;  % [G]
  end

  properties (SetAccess = private, GetAccess = {?ConfigTerrain, ?Configuration})
    surface_type (1, 1) string = "uneven";
    inclination (3, 1) double = [0.0; -120.0; 0.0];  % [deg] (-110: The robot keeps holding on to the wall)

    % Visualization
    surface_grid_color = "white";
    visualize_graspable_points (1, 1) logical = false;
  end

  %% Robot Parameters
  properties (SetAccess = private, GetAccess = {?ConfigRobot, ?Configuration})
    robot_type (1, 1) string = "HubRobo_v3_2_grip_to_palm_Fgrip10N";

    % Initial robot base position in Ground frame
    initial_base_position (3, 1) double = [0.0; 0.0; 0.08];  % [m]
    % Initial robot base orientation (euler) in Ground frame
    initial_base_orientation_euler (3, 1) double = [0.0; 0.0; 0.0];  % [deg]

    % Desired initial End-Effector (x, y) distance from robot base CoM in Ground frame
    desired_initial_EE_distance_xy_from_base_CoM (2, 1) double = [0.15; 0.15];  % [m]

    % "max_holding_force"
    gripper_detachment_detection_method (1, 1) string = "max_holding_force";
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
    max_allowable_stride (1, 1) double = 0.0;  % [m]
  end

  %% Gait Planning Parameters
  properties (SetAccess = private, GetAccess = {?ConfigGaitPlanning, ?Configuration})
    gait_type (1, 1) string = "periodic_crawl";

    % Periodic gait settings
    gait_period (1, 1) double = 4.0;  % [s]
    duty_factor (1, 1) double = 0.75;  % [0, 1]
    % Gait sequence
    % 1st dim: Limb number(s) starting at same timing during gait cycle
    % 2nd dim: Limb number(s) starting at different timing during gait cycle
    sequence uint8 = [2, 1, 3, 4];

    step_height (1, 1) double = 0.0;  % [m]
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
    proportional_gain (1, 1) double = 10.0;
    derivative_gain   (1, 1) double = 0.3;
  end

  %% Animation Settings
  properties (SetAccess = private, GetAccess = {?ConfigAnimationSettings, ?Configuration})
    display_animation (1, 1) logical = true;
    frame_rate        (1, 1) double  = 20;          % [frames/s] (positive value)
    resolution        (1, 2) double  = [320, 400];  % [px]
    show_elapsed_time (1, 1) logical = false;

    font_name (1, 1) string = "Calibri";
    font_size (1, 1) double = 15;

    % Camera related
    x_axis_limit (1, 2) double = [-0.35, 0.2];  % [m]
    y_axis_limit (1, 2) double = [-0.5, 0.5];  % [m]
    z_axis_limit (1, 2) double = [-0.5, 0.5];  % [m]
    camera_azimuth   (1, 1) double = -25;  % [deg]
    camera_elevation (1, 1) double =  10;  % [deg]
    camera_follow_robot (1, 1) logical = false;
  end

end  % config_example_demo_1
