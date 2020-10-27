%%%%%% 
%%%%%% Configuration
%%%%%% config_nonperiodic_demo_param
%%%%%% 
%%%%%% Define simulation parameters for the nonperiodic swing limb
%%%%%% selection demonstration
%%%%%% 
%%%%%% Created 2020-09-10
%%%%%% Kentaro Uno
%%%%%% Last update: 2020-09-15
%
%
% Load configurations for parameters defined for the nonperiodic gait 
%
% Function variables:
%
%     OUTPUT
%         robot_param
%         environment_param
%         gait_param
%         control_param
%         equilibrium_eval_param
%         ani_settings
%         save_settings
%         plot_settings
%     INPUT
%         -

function [robot_param, environment_param, gait_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings] = config_nonperiodic_demo_param(robot_param, environment_param, gait_param, control_param, ...
    equilibrium_eval_param, ani_settings, save_settings, plot_settings)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Type of the robot to simulate ('HubRobo_v2_2_no_grip', 'HubRobo_v2_2_grip_to_palm', 'HubRobo_v3_1_grip_to_palm', 'HubRobo_v2_2_grip_to_spine')
robot_param.robot_type = 'HubRobo_v3_1_grip_to_palm';
%%% x and y position of legs relative to base center [m]
robot_param.foot_dist  = 0.150;
%%% Height of base relative to map [m]
robot_param.base_height = 0.060;
%%% Base position [m] 2x1 vector. or 'default' for default setting
robot_param.base_pos_xy = [0.0;0.0];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Gravity [G]
environment_param.grav = 1/6;
%%% Type of the surface ('flat_HR', 'rough', 'flat_003', 'flat_006', 'flat_008', 'flat_009', 'flat_012') 
environment_param.surface_type = 'flat_009';
%%% Maximum simulation time [s]
environment_param.time_max = 20;
%%% Graspable points detection type
environment_param.graspable_points_detection_type = "all";
%%% Dynamics on/off
environment_param.dynamics_flag = 'on';
%%% Surface Inclination [deg]
environment_param.inc = 20;

%%% Gripper detachment method ('none', 'max_holding_force', 'tsm')
environment_param.detachment_detection_method = 'max_holding_force';


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Gait type ('do_nothing', 'crawl_fixed_stride','crawl_uno_ver', nonperiodic_uno_ver)
gait_param.type = 'nonperiodic_uno_ver';
%%% Step height [m]
gait_param.step_height = 0.04;
%%% Period of one cycle [s]
gait_param.T = 4;
%%% Duty cycle
gait_param.beta = 0.75;
%%% Goal position [m]
gait_param.goal = [0.4;0.4;-0.08];

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

%%% Link radius [m]
ani_settings.link_radius = 0.017;
%%% Base thickness [m]
ani_settings.base_thickness = 0.07;
%%% Robot transparency
ani_settings.robot_alpha = 0.8;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Camera related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% x-axis limits [m] [[-0.25, 0.4], [-0.25, .25], [0, .5]]
ani_settings.x_lim = [-0.25, 0.8];
%%% y-axis limits [m] [[-0.25, 0.4], [-0.25, .25]]
ani_settings.y_lim = [-0.25, 0.5];
%%% z-axis limits [m] [[-0.08, 0.1], [-0.1, .14]] [-0.25 0.25]
ani_settings.z_lim = [-0.25, 0.25];
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
ani_settings.camera_az =-40;
ani_settings.camera_el = 10;

%%% Font name
ani_settings.font_name = 'Times New Roman';
%%% Font size
ani_settings.font_size = 25;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Surface related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Surface grid color
ani_settings.grid_color = [0.9 0.9 0.9];
%%% Surface color
ani_settings.surface_color = gray;

%%% Graspable points viz. on/off
ani_settings.graspable_points_show = 'on';
    %%% Graspable points color
    ani_settings.graspable_points_color = [0.3, 0.3, 0.3];
    %%% Graspable points marker
    ani_settings.graspable_points_marker = '.';
    %%% Graspable points size
    ani_settings.graspable_points_size = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Graspable points in reachable area viz. on/off
ani_settings.graspable_points_in_reachable_area_show = 'on';

%%% Reachable area viz. on/off
ani_settings.reachable_area_show = 'on';

%%% Next desired position vis. on/off
ani_settings.next_desired_position_show = 'on';
    
%%% Plot trajectory
ani_settings.trajectory_show = 'on';
	%%% Trajectory line color
    ani_settings.trajectory_color = [0.5, 0.5, 0.5];
	%%% Trajectory line widith
    ani_settings.trajectory_width = 3;
	%%% Trajectory line type
    ani_settings.trajectory_line_type = ':';
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Support triangle visualization
ani_settings.support_triangle_show = 'on';

%%% Gravity vector visualize on/off
ani_settings.gravity_vec_show = 'on';

%%% CoM projection point vis. on/off
ani_settings.com_projection_show = 'on';

%%% Goal vis. on/off
ani_settings.goal_show = 'on';

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

end