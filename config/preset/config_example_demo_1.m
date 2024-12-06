classdef config_example_demo_1

  % Environment Parameters
  properties (SetAccess = private, GetAccess = {?ConfigWorld})
    max_simulation_time (1, 1) double = 1.0;  % [s]
    gravity (1, 1) double = 1;  % [G]
  end
  properties (SetAccess = private, GetAccess = {?ConfigTerrain})
    surface_type (1, 1) string = "uneven";
    inclination (3, 1) double = [0.0; -120.0; 0.0];  % [deg] (-110: Robot keeps holding on to wall)
  end

  % Robot Parameters
  properties (SetAccess = private, GetAccess = {?ConfigRobot})
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

end