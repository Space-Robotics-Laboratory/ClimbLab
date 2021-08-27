%%%%%% 
%%%%%% Configuration
%%%%%% config_example_demo_3_param
%%%%%% 
%%%%%% Define simulation parameters for the representative simulation cases  
%%%%%% of the ClimbLab 
%%%%%% [ ] Demo (1) Static simulation to demonstrate the detachment of the
%%%%%% gripper from an over-hanging wall
%%%%%% [ ] Demo (2) Steep slope climbing of the mammalian typed robot
%%%%%% [x] Demo (3) Perceptive walking of the half human sized quadrupedal robot
%%%%%% 
%%%%%% 
%%%%%% --------------------------------------------------------------------
%%%%%% This configration file can reproduce the similar result of CLAWAR 
%%%%%% 2021 proceedings paper by K. Uno, W. Ribeiro et al.
%%%%%% Proceedings Paper URL: 
%%%%%% https://******** (to be added)
%%%%%% --------------------------------------------------------------------
%%%%%% 
%%%%%% Created 2021-03-09
%%%%%% Keigo Haji
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
%         gripper_param
%         map_param
%         matching_settings
%         sensing_camera_param
%     INPUT
%         -

function [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param] = config_example_demo_3_param(robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Type of the robot to simulate ('HubRobo_v3_2_grip_to_palm', 'ANYmal_B', 'ALPHRED')
robot_param.robot_type = 'ALPHRED';
%%% x and y position of legs relative to base center [m]
robot_param.x_foot_dist = 0.20;
robot_param.y_foot_dist = 0.20;
%%% Height of base relative to map [m]
robot_param.base_height = 0.50;
%%% Base position [m] 2x1 vector. or 'default' for default setting
robot_param.base_pos_xy = [-0.8;0.0];
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Gravity [G]
environment_param.grav = 1;
%%% Type of the surface ('flat_HR', 'rough', 'grid_map_35mm_1m_center_thinned_50', etc. Refer to ini_surface.m) 
environment_param.surface_type = 'grid_3mx3m_dx100mm_thined_40';
%%% Maximum simulation time [s]
environment_param.time_max = 100;
%%% Graspable points detection type     'gripper' 'all' 'peaks' 0 - 100
environment_param.graspable_points_detection_type = 'all';
%%% Dynamics on/off
environment_param.dynamics_flag = 'off';
%%% Surface Inclination [deg]
environment_param.inc = 0;

%%% Gripper detachment method ('none', 'max_holding_force', 'tsm')
environment_param.detachment_detection_method = 'max_holding_force';

%%% Ground reaction force stiffness coefficient
environment_param.surface_K = 100000;
%%% Ground reaction force damping coefficient (scalar)
environment_param.surface_D = 100;

%%% Simulation stop reaching goal threshold
environment_param.sim_stop_goal_threshold = 0.10;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Gait type ('do_nothing', 'crawl_fixed_stride','crawl_gait_for_discrete_footholds', 'nonperiodic_gait_for_discrete_footholds')
gait_planning_param.type = 'nonperiodic_gait_for_discrete_footholds';
%%% Step height [m]
gait_planning_param.step_height = 0.05;
gait_planning_param.step_length = 0.10;
%%% Period of one cycle [s]
gait_planning_param.T = 4;
%%% Duty cycle
gait_planning_param.beta = 0.75;
%%% Goal position [m]
gait_planning_param.goal = [1.0;0.0;0.0];
%%% Allowable maximum stride [m]
gait_planning_param.allowable_max_stride = 0.3;
%%% Update Moving Direction on/off
gait_planning_param.local_path_plan_flag = 'off';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Control Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PD controller settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Controller Proportional (P) gain
control_param.kp = 1500;
%%% Controller Derivative (D) gain
control_param.kd = 4.5;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Equilibrium evaluation method ('none', 'tsm', 'gia')
equilibrium_eval_param.type = 'tsm';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Animation Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Link radius [m]
ani_settings.link_radius = 0.0185;
%%% Base thickness [m]
ani_settings.base_upper_thickness = 0.07;
ani_settings.base_lower_thickness = 0.03;
%%% Base horisontal scaling factor
ani_settings.base_xy_scale_factor = 1.2;
%%% Robot color
ani_settings.robot_base_upper_color = [0.1, 0.1, 0.1];
ani_settings.robot_base_lower_color = [0.1, 0.1, 0.1];
ani_settings.robot_limb_color       = [0.1, 0.1, 0.1];
%%% Robot transparency
ani_settings.robot_base_alpha = 0.8;
ani_settings.robot_limb_alpha = 0.8;

%%% Frames at each joint on/off
ani_settings.frames_show = 'off';
    ani_settings.frames_line_width = 2.0;
    ani_settings.frames_size  = 0.05;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Camera related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% x-axis limits [m] [[-0.25, 0.4], [-0.25, .25], [0, .5]]
ani_settings.x_lim = [-1.5, 1.5];
%%% y-axis limits [m] [[-0.25, 0.4], [-0.25, .25]]
ani_settings.y_lim = [-1.5, 1.5];
%%% z-axis limits [m] [[-0.08, 0.1], [-0.1, .14]] [-0.25 0.25]
ani_settings.z_lim = [-0.0, 0.7];
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
ani_settings.camera_az = -10;    %Default -40
ani_settings.camera_el = 20;    %default 10

%%% Font name
ani_settings.font_name = 'Calibri';
%%% Font size
ani_settings.font_size = 25;

%%% Frame rate [frames/s] [20,25,50]
ani_settings.frame_rate = 10;
%%% Animation video resolution [px] [[1280 720], [640 480]]
ani_settings.animation_resolution = [1280 720];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Surface related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Surface viz. on/off
ani_settings.surface_show = 'on';
    %%% Surface grid color
    ani_settings.grid_color = 'none';
    %%% Surface color
    ani_settings.surface_color = gray;

%%% Graspable points viz. on/off
ani_settings.graspable_points_show = 'on';
    %%% Graspable points color
    ani_settings.graspable_points_color = [0.8, 0.8, 0.8];
    %%% Graspable points alpha
    ani_settings.graspable_points_alpha = 1.0;
    %%% Graspable points marker
    ani_settings.graspable_points_marker = 'o';
    %%% Graspable points size
    ani_settings.graspable_points_size = 20;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Graspable points in reachable area viz. on/off
ani_settings.graspable_points_in_reachable_area_show = 'off';

%%% Reachable area viz. on/off
ani_settings.reachable_area_show = 'on';
    %%% Reachable area line color
    ani_settings.reachable_area_line_color = 'm';
    %%% Reachable area line width
    ani_settings.reachable_area_line_width = 1;


%%% Moving Direction viz. on/off
ani_settings.moving_direction_show = 'off';
    ani_settings.moving_direction_vec_color = [1 0 0];
    ani_settings.moving_direction_vec_width = 3;
    ani_settings.moving_direction_expansion_factor = 0.08;


%%% Next desired position vis. on/off
ani_settings.next_desired_position_show = 'on';
    
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
    %%% Gravitational Acceleration vector visualize on/off
    ani_settings.gravitational_acceleration_vec_show = 'on';
        %%% Gravity Acceleration vector color
        ani_settings.gravitational_acceleration_vec_color = [1 0 0];
        %%% Gravity Acceleration vector width
        ani_settings.gravitational_acceleration_vec_width = 3;
        
%%% Transformation from force to visualize Fg (gravitational force) and 
%%% Fe (reaction force) Stable Region in the position coordinate 
%%% -> this scale is used for all force-dimensional variables visualization
ani_settings.force_expansion_factor = 0.005;
    ani_settings.reaction_force_show = 'off';
    
%%% CoM projection point vis. on/off
ani_settings.com_projection_show = 'off';

%%% Goal vis. on/off
ani_settings.goal_show = 'on';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Sensing Camera related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% RealSense Camera fov vis. on/off
ani_settings.sensing_fov_show = 'on';
    %%% Sensing camera position marker size
    ani_settings.sensing_camera_MarkerSize = 1;
    %%% Sensing camera marker color
    ani_settings.sensing_camera_MarkerColor = '.k';
    %%% Sensing camera fov line width
    ani_settings.sensing_camera_fov_LineWidth = 1;
    %%% Sensing camera fov line color
    ani_settings.sensing_camera_fov_LineColor = 'w';
%%% RealSense Camera fov filling vis. on/off
ani_settings.sensing_fov_face_filling = 'on';
    %%% Filling color
    ani_settings.sensing_fov_face_color = 'w';
    %%% Filling color alpha
    ani_settings.sensing_fov_face_alpha = 0.2;
    
%%% Sensed Graspable points viz. on/off
ani_settings.sensed_graspable_points_show = 'on';
    %%% Graspable points color
    ani_settings.sensed_graspable_points_color = [0 136/255 170/255];
    %%% Graspable points alpha
    ani_settings.sensed_graspable_points_alpha = 1.0;
    %%% Graspable points marker
    ani_settings.sensed_graspable_points_marker = 'o';
    %%% Graspable points size
    ani_settings.sensed_graspable_points_size = 20;

    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Save Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Time interval for saving variables (should be larger than time-step)
save_settings.variable_saving_time_interval = 0.05;

%%% Save basic variables to csv file on/off
save_settings.csv_file = 'on';

%%% Save TSM variables to csv file on/off
save_settings.tsm = 'on';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Plot footholds history on/off
plot_settings.footholds = 'on';
%%% Plot TSM
plot_settings.tsm = 'on';

plot_settings.joint_torque = 'on';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Sensing Camera Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Sensing Camera on/off 
sensing_camera_param.sensing_flag =  'on';
%%% Sensing type
sensing_camera_param.sensing_type =  'RealSense_d435i';  % 'circle' or 'RealSense_d435i'
%%% Known area shape
sensing_camera_param.known_area_shape = 'circle';   %'rectangle' or 'circle'

sensing_camera_param.mounting_angle =  [0; -60; 0]*pi/180;  % rpy
sensing_camera_param.mounting_position = [0.08599/sqrt(2); 0 ; 0.07];    %[m] 

sensing_camera_param.fov_min_distance = 0.0;    %[m]

%%% Circle Param
sensing_camera_param.ini_circular_radius_from_base_pos = 0.6;    %[m]


end