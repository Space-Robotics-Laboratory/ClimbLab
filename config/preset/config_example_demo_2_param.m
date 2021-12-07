%%%%%% 
%%%%%% Configuration
%%%%%% config_example_demo_2_param
%%%%%% 
%%%%%% Define simulation parameters for the representative simulation cases  
%%%%%% of the ClimbLab 
%%%%%% [ ] Demo (1) Static simulation to demonstrate the detachment of the
%%%%%% gripper from an over-hanging wall
%%%%%% [x] Demo (2) Steep slope climbing of the mammalian typed robot
%%%%%% [ ] Demo (3) Perceptive walking of the half human sized quadrupedal robot
%%%%%% 
%%%%%% 
%%%%%% --------------------------------------------------------------------
%%%%%% This configration file can reproduce the similar result of CLAWAR 
%%%%%% 2021 proceedings paper by K. Uno, W. Ribeiro et al.
%%%%%% Proceedings Paper URL: 
%%%%%% https://******** (to be added)
%%%%%% --------------------------------------------------------------------
%%%%%% 
%%%%%% Creation: 2021-03-02
%%%%%% Kentaro Uno
%%%%%% Last update: 2021-06-30
%%%%%% Kentaro Uno
%
%
% Function variables:
%
%     OUTPUT
%         robot_param
%         environment_param
%         gait_planning_param
%         control_param
%         equilibrium_eval_param
%         ani_settings
%         save_settings
%         plot_settings
%     INPUT
%         -

function [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param] = config_example_demo_2_param(robot_param, environment_param, gait_planning_param, control_param, ...
    equilibrium_eval_param, ani_settings, save_settings, plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Type of the robot to simulate ('HubRobo_v2_2_no_grip', 'HubRobo_v2_2_grip_to_palm', 'HubRobo_v3_1_grip_to_palm', 'HubRobo_v2_2_grip_to_spine')
robot_param.robot_type = 'ANYmal_B';

% Nomally, the following four items should be defined in the LP file, 
% but it is also possible to specify the joint allocation type and offset 
% angles (theta1 and 2) for the hip joints here for more convenience.

% if you set "insect" for the following type, theta1 and 2 are forced to be pi/4 and -pi/2 [rad],
% respectively.
robot_param.joint_allocation_type = 'mammal'; % this can also be set in ini_LP file
% for the mammalian robot, you can specify the leg configuration from "oo", "xx", and "M".
robot_param.leg_config_type = 'xx'; % this can also be set in LP file
robot_param.theta_1 = 0.0; % [rad]
% note that if you put the negative num for theta_2, the IK solver take automatically
% 'oo' configuration solution
robot_param.theta_2 = 0.0; % [rad]

%%% "Initial" x and y position of legs relative to base center [m]
robot_param.x_foot_dist = 0.35;
robot_param.y_foot_dist = 0.35;
%%% Height of base relative to map [m]
robot_param.base_height = 0.25;

%%% Base position [m] 2x1 vector. or 'default' for default setting
robot_param.base_pos_xy = [0.0;0.0];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Sensing Camera Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Following Values are written in RealSense D435i datasheet.
sensing_camera_param.fov_horizontal = 86*pi/180;     %[radian]
sensing_camera_param.fov_vertical = 57*pi/180;   %[radian]
sensing_camera_param.fov_max_distance = 2;   %[m]
sensing_camera_param.fov_min_distance = 0.05;    %[m]

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Gravity [G]
environment_param.grav = 1;
%%% Type of the surface ('flat_HR', 'flat_HR_5m_x_5m', 'rough', 'rough_3m_x_3m', 'climbing_holds_1m_x_1m', 'flat_003', 'flat_006', 'flat_008', 'flat_009', 'flat_012') 
environment_param.surface_type = 'flat_HR_5m_x_5m';
%%% Maximum simulation time [s]
environment_param.time_max = 16;
%%% Dynamics on/off
environment_param.dynamics_flag = 'on';
%%% Surface Inclination [deg]
environment_param.inc = 45;

%%% Floor reaction force stiffness coefficient
environment_param.surface_K = 100000;
%%% Floor reaction force damping coefficient
environment_param.surface_D = 100.0;  

%%% Gripper detachment method ('none', 'max_holding_force', 'tsm')
environment_param.detachment_detection_method = 'max_holding_force';


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Gait type (e.g. 'do_nothing', 'diagonal_gait_fixed_stride')
gait_planning_param.type = 'diagonal_gait_fixed_stride';
%%% Step height [m]
gait_planning_param.step_height = 0.1;
gait_planning_param.step_length = 0.1;
%%% Period of one cycle [s]
gait_planning_param.T = 8;
%%% Duty cycle
gait_planning_param.beta = 0.75;
%%% Goal position [m]
gait_planning_param.goal = [0.4;0.5;0.0];
%%% Allowable maximum stride [m]
gait_planning_param.allowable_max_stride = 0.2; 
%%% Walking sequence:
gait_planning_param.sequence = [1; 3; 4; 2];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Control Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Controller type ('do_nothing', 'torque_PD')
control_param.type = 'torque_PD';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PD controller settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Controller Proportional (P) gain
control_param.kp = 850.0;
%%% Controller Derivative (D) gain
control_param.kd = 3.0;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Equilibrium evaluation method ('none', 'tsm', 'gia', 'tsm_and_gia')
equilibrium_eval_param.type = 'tsm_and_gia';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Animation Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% elapsed time show on/off
ani_settings.animation_elapsed_time_show = 'off';

%%% Frame rate [frames/s] [20,25,50]
ani_settings.frame_rate = 20;
%%% Animation video resolution [px] [[1280 720], [640 480]]
ani_settings.animation_resolution = [640 480];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Camera related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% x-axis limits [m] [[-0.25, 0.4], [-0.25, .25], [0, .5]]
ani_settings.x_lim = [-0.5, 1.0];
%%% y-axis limits [m] [[-0.25, 0.4], [-0.25, .25]]
ani_settings.y_lim = [-1.0, 1.0];
%%% z-axis limits [m] [[-0.08, 0.1], [-0.1, .14]] [-0.25 0.25]
ani_settings.z_lim = [-0.5, 1.0];
%%% Lower z-axis limit same as lower point on surface on/off
ani_settings.z_lim_low_is_surface = 'off';

%%% 'on' camera follows robot, 'off' camera fixed
ani_settings.move_camera = 'off';
    %%% x-axis visualization range [m] [[-0.25, 0.4], [-0.25, .25], [0, .5]]
    ani_settings.x_vis_range = ani_settings.x_lim - mean(ani_settings.x_lim);
    %%% y-axis visualization range [m] [[-0.25, 0.4], [-0.25, .25]]
    ani_settings.y_vis_range = ani_settings.y_lim - mean(ani_settings.y_lim);
    %%% z-axis visualization range [m] [[-0.08, 0.1], [-0.1, .14]]
    ani_settings.z_vis_range = ani_settings.z_lim - mean(ani_settings.z_lim);
%%% Camera angle azimuth and elevation [deg]
ani_settings.camera_az = -25;
ani_settings.camera_el =  10;

%%% Font name
ani_settings.font_name = 'Calibri';
%%% Font size
ani_settings.font_size = 20;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Surface related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Surface viz. on/off
ani_settings.surface_show = 'on';
    %%% Surface grid color
    ani_settings.grid_color = 'white';
    %%% Surface color
    ani_settings.surface_color = [0.5, 0.5, 0.5];

%%% Graspable points viz. on/off
ani_settings.graspable_points_show = 'off';
    %%% Graspable points color
    ani_settings.graspable_points_color = [0.0, 0.8, 0.0];
    %%% Graspable points marker
    ani_settings.graspable_points_marker = '.';
    %%% Graspable points size
    ani_settings.graspable_points_size = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Link radius [m]
ani_settings.link_radius = 0.030;
%%% Base thickness [m]
ani_settings.base_upper_thickness = 0.20;
ani_settings.base_lower_thickness = 0.05;
%%% Base horisontal scaling factor
ani_settings.base_xy_scale_factor = 1.0;
%%% Robot color
ani_settings.robot_base_upper_color = [0.0, 0.3, 0.6];
ani_settings.robot_base_lower_color = [0.3, 0.3, 0.3];
ani_settings.robot_limb_color = [0.3, 0.3, 0.3];
%%% Robot transparency
ani_settings.robot_base_alpha = 1.0;
ani_settings.robot_limb_alpha = 1.0;

%%% Frames at each joint on/off
ani_settings.frames_show = 'off';
    ani_settings.frames_line_width = 2.0;
    ani_settings.frames_size = 0.1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Graspable points in reachable area viz. on/off
ani_settings.graspable_points_in_reachable_area_show = 'off';
    %%% Graspable points size
    ani_settings.graspable_points_in_reachable_area_size = 40;

%%% Reachable area viz. on/off
ani_settings.reachable_area_show = 'off';
    %%% Reachable area line width
    ani_settings.reachable_area_line_width = 3;
    
%%% Next desired position vis. on/off
ani_settings.next_desired_position_show = 'off';
    ani_settings.next_desired_position_color = [0.0, 0.7, 0.7];
    %%% Next desired position line width
    ani_settings.next_desired_position_line_width = 3;
    
%%% Plot trajectory
ani_settings.trajectory_show = 'off';
	%%% Trajectory line color
    ani_settings.trajectory_color = [0.5, 0.5, 0.5];
	%%% Trajectory line widith
    ani_settings.trajectory_width = 3;
	%%% Trajectory line type
    ani_settings.trajectory_line_type = ':';
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Support triangle visualization
ani_settings.support_triangle_show = 'on';
    %%% Support triangle color
    ani_settings.support_triangle_color = [0 136/255 170/255];
    %%% Support triangle transparency
    ani_settings.support_triangle_edge_color = 'none';
    
%%% Transformation from acceleration to visualize GIA Stable Region, GIA
%%% vector in the position coordinate -> this scale is used for all 
%%% accelerational variables visualization
ani_settings.acceleration_expansion_factor = 0.02;

    %%% GIA Stable Region visualize on/off (requires equilibrium evaluation method as 'gia')
    ani_settings.gia_stable_region_show = 'on';
        %%% Stability polyhedron face color
        ani_settings.gia_stable_region_Face_color = [0 136/255 170/255];
        %%% Stability polyhedron face alpha
        ani_settings.gia_stable_region_Face_alpha = 0.2;
        %%% Stability polyhedron edge color
        ani_settings.gia_stable_region_Edge_color = 'none';
        %%% Stability polyhedron edge width
        ani_settings.gia_stable_region_Edge_width = 1;
        
    %%% Gravitational Acceleration vector visualize on/off
    ani_settings.gravitational_acceleration_vec_show = 'off';
        %%% Gravity Acceleration vector color
        ani_settings.gravitational_acceleration_vec_color = [1.0, 0.0, 0.0];
        %%% Gravity Acceleration vector width
        ani_settings.gravitational_acceleration_vec_width = 6;

    %%% GIA vector visualize on/off
    ani_settings.gia_vec_show = 'on';
        %%% GIA vector color
        ani_settings.gia_vec_color = [1 0 0];
        %%% GIA vector width
        ani_settings.gia_vec_width = 3;

%%% Transformation from force to visualize Fg (gravitational force) and 
%%% Fe (reaction force) Stable Region in the position coordinate 
%%% -> this scale is used for all force-dimensional variables visualization
ani_settings.force_expansion_factor = 0.005;
    
    %%% Gravitational Force vector visualize on/off
    ani_settings.gravitational_force_vec_show = 'off';
        %%% Gravity vector color
        ani_settings.gravitational_force_vec_color = [0.7 0 0];
        %%% Gravity vector width
        ani_settings.gravitational_force_vec_width = 3;    
    
    %%% Reaction force vectors vis. on/off
    ani_settings.reaction_force_show = 'off';
        %%% Reaction force vectors color
        ani_settings.reaction_force_vec_color = [0.7 0 0.7];
        %%% Reaction force vectors color width
        ani_settings.reaction_force_vec_width = 2;
        
%%% CoM projection point vis. on/off
ani_settings.com_projection_show = 'off';
    %%% CoM projection point vis. marker size
    ani_settings.com_projection_size = 40;
    %%% CoM projection point vis. height from the ground
    % visualization position can be offseted to locate it vertically upper to the ground to show it well
    ani_settings.com_projection_vis_height = 0.04; % [m]
    
%%% Goal vis. on/off
ani_settings.goal_show = 'off';
    %%% Goal vis. marker size
    ani_settings.goal_size  = 40;
    %%% Goal vis. height from the ground
    ani_settings.goal_vis_height = 0.04; % [m]
    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Sensing Camera related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% RealSense Camera fov vis. on/off
ani_settings.sensing_fov_show = 'off';
    %%% Sensing camera position marker size
    ani_settings.sensing_camera_MarkerSize = 10;
    %%% Sensing camera marker color
    ani_settings.sensing_camera_MarkerColor = '.k';
    %%% Sensing camera fov line width
    ani_settings.sensing_camera_fov_LineWidth = 1;
    %%% Sensing camera fov line color
    ani_settings.sensing_camera_fov_LineColor = 'w';
%%% RealSense Camera fov filling vis. on/off
ani_settings.sensing_fov_face_filling = 'off';
    %%% Filling color
    ani_settings.sensing_fov_face_color = 'w';
    %%% Filling color alpha
    ani_settings.sensing_fov_face_alpha = 0.5;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Save Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Time interval for saving variables (should be larger than time-step)
save_settings.variable_saving_time_interval = 0.05;

%%% Save basic variables to csv file on/off
save_settings.csv_file = 'on';
%%% Save TSM variables to csv file on/off (requires equilibrium evaluation method as 'tsm')
save_settings.tsm = 'on';
%%% Save GIA variables to csv file on/off (requires equilibrium evaluation method as 'gia')
save_settings.gia = 'on';
%%% Save manipularbility measure to csv file on/off 
save_settings.manipulability = 'off';
%%% Save Maximum of absolute torque of all joint
save_settings.joint_max_torque = 'on';
%%% Save Root Mean Square (RMS of torque of all joint (*need Signal Processing Toolbox)
save_settings.joint_rms_torque = 'on';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot Base position
plot_settings.base_pos = 'off';
%%% Plot Joint torque
plot_settings.joint_torque = 'on';
%%% Plot TSM  (requires equilibrium evaluation method as 'tsm')
plot_settings.tsm = 'on';
%%% Plot GIA (Acceleration) margin (requires equilibrium evaluation method as 'gia')
plot_settings.gia_margin = 'on';
%%% Plot GIA Inclination margin (requires equilibrium evaluation method as 'gia')
plot_settings.gia_inclination_margin = 'on';
%%% Plot manipulability
plot_settings.manipulability = 'off';
%%% Plot max torque of all joint
plot_settings.joint_max_torque = 'on';
%%% Plot RMS torque of all joint
plot_settings.joint_rms_torque = 'on';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gripper Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Map Parameters and settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Matching Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
matching_settings.submatching = 'on';              
matching_settings.colony = 'on';        
matching_settings.threshold = 130; 

end