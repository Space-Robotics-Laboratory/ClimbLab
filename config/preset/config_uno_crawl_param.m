%%%%%% Configuration
%%%%%% config_uno_crawl_param
%%%%%% 
%%%%%% Define uno_crawl_gait parameters
%%%%%% 
%%%%%% Created 2020-07-09
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-09
%
%
% Load configurations for parameters defined for uno crawl fait simulation 
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
    plot_settings] = config_uno_crawl_param(robot_param, environment_param, gait_param, control_param, ...
    equilibrium_eval_param, ani_settings, save_settings, plot_settings)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Type of the robot to simulate ('HubRobo_no_grip', 'HubRobo_grip_to_palm', 'HubRobo_grip_to_spine')
robot_param.robot_type = 'HubRobo_grip_to_palm';
%%% x and y position of legs relative to base center [m]
robot_param.foot_dist  = 0.14;
%%% Height of base relative to map [m]
robot_param.base_height = 0.09;
%%% Base position [m] 2x1 vector. or 'default' for default setting
robot_param.base_pos_xy = [0;0];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Type of the surface ('flat_HR', 'rough', 'flat_003', 'flat_006', 'flat_008', 'flat_009', 'flat_012') 
environment_param.surface_type = 'flat_003';
%%% Maximum simulation time [s]
environment_param.time_max = 30;
%%% Graspable points detection type
environment_param.graspable_points_detection_type = 30;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Gait type ('do_nothing', 'crawl_fixed_stride','crawl_uno_ver')
gait_param.type = 'crawl_uno_ver';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Control Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Equilibrium evaluation method ('none', 'tsm', 'gia')
equilibrium_eval_param.type = 'tsm';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Animation Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Link radius [m]
ani_settings.link_radius = 0.012;
%%% Base thickness [m]
ani_settings.base_thickness = 0.05;
%%% Robot transparency
ani_settings.robot_alpha = 1;

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

%%% Save TSM variables to csv file on/off
save_settings.tsm = 'on';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot footholds history on/off
plot_settings.footholds = 'on';
%%% Plot TSM
plot_settings.tsm = 'on';

end