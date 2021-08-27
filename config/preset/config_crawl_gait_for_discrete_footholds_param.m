%%%%%% Configuration
%%%%%% config_crawl_gait_for_discrete_footholds_param
%%%%%% 
%%%%%% Define crawl_gait_for_discrete_footholds parameters
%%%%%% 
%%%%%% Created 2020-07-09
%%%%%% Warley Ribeiro
%%%%%% Last updated: 2021-07-09
%%%%%% Keigo Haji
%
%
% Load configurations for parameters defined for periodic gait for discrete
% footholds simulation 
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
    plot_settings] = config_crawl_gait_for_discrete_footholds_param(robot_param, environment_param, gait_planning_param, control_param, ...
    equilibrium_eval_param, ani_settings, save_settings, plot_settings)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Type of the robot to simulate ('HubRobo_v2_2_no_grip', 'HubRobo_v2_2_grip_to_palm', 'HubRobo_v3_1_grip_to_palm', 'HubRobo_v2_2_grip_to_spine')
robot_param.robot_type = 'HubRobo_v3_2_grip_to_palm';
%%% x and y position of legs relative to base center [m]
robot_param.x_foot_dist  = 0.18;
robot_param.y_foot_dist  = 0.18;
%%% Height of base relative to map [m]
robot_param.base_height = 0.10;
%%% Base position [m] 2x1 vector. or 'default' for default setting
robot_param.base_pos_xy = [0;0];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Type of the surface ('flat_HR', 'rough', 'flat_003', 'flat_006', 'flat_008', 'flat_009', 'flat_012') 
environment_param.surface_type = 'flat_003';
%%% Maximum simulation time [s]
environment_param.time_max = 8;
%%% Graspable points detection type
environment_param.graspable_points_detection_type = 0;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Gait type ('do_nothing', 'crawl_fixed_stride','crawl_gait_for_discrete_footholds','nonperiodic_gait_for_discrete_footholds' )
gait_planning_param.type = 'crawl_gait_for_discrete_footholds';
%%% Step height [m]
gait_planning_param.step_height = 0.03;
%%% Period of one cycle [s]
gait_planning_param.T = 4;
%%% Allowable maximum stride [m]
gait_planning_param.allowable_max_stride = 0.1; 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Control Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PD controller settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Controller Proportional (P) gain
control_param.kp = 10.0;
%%% Controller Derivative (D) gain
control_param.kd = 0.30;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Equilibrium evaluation method ('none', 'tsm', 'gia')
equilibrium_eval_param.type = 'tsm';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Animation Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Link radius [m]
ani_settings.link_radius = 0.015;
%%% Base thickness [m]
ani_settings.base_upper_thickness = 0.04;
ani_settings.base_lower_thickness = 0.02;
%%% Base horisontal scaling factor
ani_settings.base_xy_scale_factor = 1.2;
%%% Robot transparency
ani_settings.robot_base_alpha = 1;
ani_settings.robot_limb_alpha = 1;

%%% Graspable points viz. on/off
ani_settings.graspable_points_show = 'on';
    %%% Graspable points size
    ani_settings.graspable_points_size = 8;
%%% Graspable points in reachable area viz. on/off
ani_settings.graspable_points_in_reachable_area_show = 'on';
%%% Reachable area viz. on/off
ani_settings.reachable_area_show = 'on';
%%% Goal vis. on/off
ani_settings.goal_show = 'on';
%%% Next desired position vis. on/off
ani_settings.next_desired_position_show = 'on';
%%% Support triangle visualization
ani_settings.support_triangle_show = 'on';
%%% CoM projection point vis. on/off
ani_settings.com_projection_show = 'on';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Save Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Save basic variables to csv file on/off
save_settings.csv_file = 'on';

%%% Save TSM variables to csv file on/off (requires equilibrium evaluation method as 'tsm')
save_settings.tsm = 'on';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot footholds history on/off
plot_settings.footholds = 'on';
%%% Plot TSM
plot_settings.tsm = 'on';

end