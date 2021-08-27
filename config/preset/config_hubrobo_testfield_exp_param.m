%%%%%% 
%%%%%% Configuration
%%%%%% config_hubrobo_testfield_exp_param
%%%%%% 
%%%%%% Define simulation parameters for the target detection and nonperiodic 
%%%%%% swing limb selection on the hubrobo testfield
%%%%%%  
%%%%%% Creation: 2021-04-12
%%%%%% Keigo Haji
%%%%%% Last update: 2021-04-16
%%%%%% Keigo Haji
%
%
% Load configurations for parameters to simulate HubRobo experiment on the
% bouldering holds testfield
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
    plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param] = config_hubrobo_testfield_exp_param(robot_param, environment_param, gait_planning_param, control_param, ...
    equilibrium_eval_param, ani_settings, save_settings, plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Type of the robot to simulate ('HubRobo_v2_2_no_grip', 'HubRobo_v2_2_grip_to_palm', 'HubRobo_v3_1_grip_to_palm', 'HubRobo_v2_2_grip_to_spine')
robot_param.robot_type = 'HubRobo_v3_2_grip_to_palm';
%%% x and y position of legs relative to base center [m]
robot_param.x_foot_dist = 0.150;
robot_param.y_foot_dist = 0.150;
%%% Height of base relative to map [m]
    % robot_param.base_height = 0.060;
    robot_param.base_height = 0.080;

%%% Base position [m] 2x1 vector
    robot_param.base_pos_xy = [-0.4;0.1];   
%     robot_param.base_pos_xy = [-0.6;-0.3];
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Sensing Camera Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Following Values are written in RealSense D435i datasheet.
sensing_camera_param.fov_horizontal = 86*pi/180;     %[radian]
sensing_camera_param.fov_vertical = 57*pi/180;   %[radian]
sensing_camera_param.fov_max_distance = 0.3;   %[m]
sensing_camera_param.fov_min_distance = 0.05;    %[m]

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Gravity [G]
environment_param.grav = 1/6;

%%% Type of the surface ('flat_HR', 'rough', 'climbing_holds_1m_x_1m', 'flat_003', 'flat_006', 'flat_008', 'flat_009', 'flat_012') 
environment_param.surface_type = 'climbing_holds_map_on_full_testfield';
    if  strcmp(environment_param.surface_type, 'climbing_holds_map_on_full_testfield')
        robot_param.base_pos_xy = [-0.6;-0.3];
    end
 

%%% Maximum simulation time [s]
environment_param.time_max = 96;

%%% Graspable points detection type 'gripper' or 'climbing_holds_map_on_testfield'
    % If 'climbing_holds_map_on_testfield' is selected, you can skip the
    % target_detection, and get the result directly.
    environment_param.graspable_points_detection_type = 'climbing_holds_map_on_full_testfield';

%%% Dynamics on/off
environment_param.dynamics_flag = 'off';

%%% Surface Inclination [deg]
    environment_param.inc = 0;
    % environment_param.inc = 30;
    % environment_param.inc = 45;
    % environment_param.inc = 60;
    % environment_param.inc = 75;
    % environment_param.inc = 90;

%%% Gripper detachment method ('none', 'max_holding_force', 'tsm')
environment_param.detachment_detection_method = 'max_holding_force';

%%% Simulation stop reaching goal threshold
    % environment_param.sim_stop_goal_threshold = 0.03;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Gait type ('do_nothing', 'crawl_fixed_stride','crawl_gait_for_discrete_footholds', nonperiodic_gait_for_discrete_footholds)
gait_planning_param.type = 'nonperiodic_gait_for_discrete_footholds';

%%% Step height [m]
gait_planning_param.step_height = 0.04;

%%% Period of one cycle [s]
    if strcmp(environment_param.dynamics_flag, 'on')
        gait_planning_param.T = 16;
    end 
    if strcmp(environment_param.dynamics_flag, 'off')
        gait_planning_param.T = 4;
    end 

%%% Duty cycle
gait_planning_param.beta = 0.75;

%%% Goal position [m]
    gait_planning_param.goal = [0.4;-0.1;0.0];
    if  strcmp(environment_param.surface_type, 'climbing_holds_map_on_full_testfield')
        gait_planning_param.goal = [0.8;0.3;0.0];
    end


%%% Allowable maximum stride [m]
    % gait_planning_param.allowable_max_stride = 0.1; 
    gait_planning_param.allowable_max_stride = 0.5; 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Control Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PD controller settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Controller Proportional (P) gain
control_param.kp = 10.0;
%%% Controller Derivative (D) gain
control_param.kd = 0.30;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Equilibrium evaluation method ('none', 'tsm', 'gia')
equilibrium_eval_param.type = 'tsm';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Animation Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Frame rate [frames/s] [20,25,50]
ani_settings.frame_rate = 20;
%%% Animation video resolution [px] [[1280 720], [640 480]]
ani_settings.animation_resolution = [1280 720];

%%% Link radius [m]
ani_settings.link_radius = 0.0185;
%%% Base thickness [m]
ani_settings.base_upper_thickness = 0.07;
ani_settings.base_lower_thickness = 0.03;
%%% Base horisontal scaling factor
ani_settings.base_xy_scale_factor = 1.05;
%%% Robot color
ani_settings.robot_base_upper_color = [0.1, 0.1, 0.1];
ani_settings.robot_base_lower_color = [0.1, 0.1, 0.1];
ani_settings.robot_limb_color       = [0.1, 0.1, 0.1];
%%% Robot transparency
ani_settings.robot_base_alpha = 0.72;
ani_settings.robot_limb_alpha = 0.72;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Camera related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% x-axis limits [m] [[-0.25, 0.4], [-0.25, .25], [0, .5]]
ani_settings.x_lim = [-0.6, 0.6];
%%% y-axis limits [m] [[-0.25, 0.4], [-0.25, .25]]
ani_settings.y_lim = [-0.5, 0.5];

    if  strcmp(environment_param.surface_type, 'climbing_holds_map_on_full_testfield')
        ani_settings.x_lim = [-0.85, 1];
        ani_settings.y_lim = [-0.6, 0.6];
    end

%%% z-axis limits [m] [[-0.08, 0.1], [-0.1, .14]] [-0.25 0.25]
ani_settings.z_lim = [-0.05, 0.30];
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
ani_settings.camera_az = -40;
ani_settings.camera_el =  20;

%%% Font name
ani_settings.font_name = 'Times New Roman';
%%% Font size
ani_settings.font_size = 25;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Surface related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Surface viz. on/off
ani_settings.surface_show = 'on';
    %%% Surface grid color
    ani_settings.grid_color = [0.9 0.9 0.9];
    %%% Surface color
    ani_settings.surface_color = gray;

%%% Graspable points viz. on/off
ani_settings.graspable_points_show = 'on';
    %%% Graspable points color
    ani_settings.graspable_points_color = [0.0, 1.0, 0.0];
    %%% Graspable points marker
    ani_settings.graspable_points_marker = 'o';
    %%% Graspable points size
    ani_settings.graspable_points_size = 20;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Graspable points in reachable area viz. on/off
ani_settings.graspable_points_in_reachable_area_show = 'on';
    %%% Graspable points size
    ani_settings.graspable_points_in_reachable_area_size = 40;

%%% Reachable area viz. on/off
ani_settings.reachable_area_show = 'on';
    %%% Reachable area line width
    ani_settings.reachable_area_line_width = 3;
    
%%% Next desired position vis. on/off
ani_settings.next_desired_position_show = 'on';
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
    %%% Support triangle transparency
    ani_settings.support_triangle_edge_color = 'none';
    
%%% Transformation from acceleration to visualize GIA Stable Region, GIA
%%% vector in the position coordinate -> this scale is used for all 
%%% accelerational variables visualization
ani_settings.acceleration_expansion_factor = 0.02;
    %%% Gravitational Acceleration vector visualize on/off
    ani_settings.gravitational_acceleration_vec_show = 'on';
        %%% Gravity Acceleration vector color
        ani_settings.gravitational_acceleration_vec_color = [1.0, 0.0, 0.0];
        %%% Gravity Acceleration vector width
        ani_settings.gravitational_acceleration_vec_width = 6;
        
%%% CoM projection point vis. on/off
ani_settings.com_projection_show = 'on';
    %%% CoM projection point vis. marker size
    ani_settings.com_projection_size = 40;
    %%% CoM projection point vis. height from the ground
    % visualization position can be offseted to locate it vertically upper to the ground to show it well
    ani_settings.com_projection_vis_height = 0.04; % [m]
    
%%% Goal vis. on/off
ani_settings.goal_show = 'on';
    %%% Goal vis. marker size
    ani_settings.goal_size  = 40;
    %%% Goal vis. height from the ground
    ani_settings.goal_vis_height = 0.04; % [m]

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Sensing Camera related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% RealSense Camera fov vis. on/off
ani_settings.sensing_fov_show = 'off';
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
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Save Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gripper Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Map Parameters and settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Matching Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
matching_settings.submatching = 'on';              
matching_settings.colony = 'on';        
matching_settings.threshold = 120;

% By this switch, the targets detected on the board will be deleted
matching_settings.delete_lower_targets = 'on';


end