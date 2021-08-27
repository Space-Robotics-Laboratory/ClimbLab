%%%%%% Configuration
%%%%%% config_base_pos_opt_quasistatic_param
%%%%%% 
%%%%%% Define config_base_pos_opt_quasistatic_param parameters
%%%%%% 
%%%%%% Created 2020-07-09
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2021-03-24
%%%%%% Keigo Haji
%
%
% Load the parameters of the quasistatic simulation to optimize the base
% pose based on the GIA of the robot and the manipulability of the legs. 
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
    plot_settings, gripper_param, map_param, matching_settings] = config_base_pos_opt_quasistatic_param(robot_param, environment_param, gait_planning_param, control_param, ...
    equilibrium_eval_param, ani_settings, save_settings, plot_settings, gripper_param, map_param, matching_settings)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Type of the robot to simulate ('HubRobo_no_grip', 'HubRobo_grip_to_palm', 'HubRobo_grip_to_spine', 'HubRobo_grip_to_spine_old')
robot_param.robot_type = 'HubRobo_v3_2_grip_to_palm';
%%% x and y position of legs relative to base center [m]
robot_param.x_foot_dist = 0.15;
robot_param.y_foot_dist = 0.15;

%%% Height of base relative to map [m]
if strcmp(robot_param.robot_type,'HubRobo_no_grip')
    robot_param.base_height = 0.10;
end
if strcmp(robot_param.robot_type,'HubRobo_grip_to_spine_old')
    robot_param.base_height = 0.14;
end
if strcmp(robot_param.robot_type,'HubRobo_v3_2_grip_to_palm')
    robot_param.base_height = 0.1;
end
%%% Base position [m] 2x1 vector. or 'default' for default setting
robot_param.base_pos_xy = [0;0];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Type of the surface ('flat_HR', 'rough', 'flat_003', 'flat_006', 'flat_008', 'flat_009', 'flat_012') 
environment_param.surface_type = 'flat_003';
%%% Maximum simulation time [s]
environment_param.time_max = 4;%500*N;
%%% Graspable points detection type
environment_param.graspable_points_detection_type = 0;
environment_param.inc = 0;
environment_param.dynamics_flag = 'no_acc';
% environment_param.grav = 1;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Gait type ('do_nothing', 'crawl_fixed_stride','crawl_gait_for_discrete_footholds','nonperiodic_gait_for_discrete_footholds','GIA_opt_based_pose_planner')
gait_planning_param.type = 'GIA_opt_based_pose_planner';
gait_planning_param.step_height = 0.04;

gait_planning_param.T = 4;
% gait_planning_param.goal = [0.7;0;0.175];
% gait_planning_param.goal = [0.36;0;0.5436];
gait_planning_param.goal = [0.6;0;0.2];

% [a,b,c] = get_map_pos(gait_planning_param.goal(1),gait_planning_param.goal(2));
% gait_planning_param.goal = [a;b;c];

%%% Allowable maximum stride [m]
gait_planning_param.allowable_max_stride = 0.1; 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Control Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if strcmp(robot_param.robot_type, 'HubRobo_v3_1_grip_to_palm')
    control_param.kp = 10.0;
    control_param.kd = 0.3;
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Equilibrium evaluation method ('none', 'tsm', 'gia')
equilibrium_eval_param.type = 'gia';
if strcmp(equilibrium_eval_param.type,'gia')
    ani_settings.gia_stable_region_show = 'on';
    plot_settings.gia = 'on';
else
    ani_settings.gia_stable_region_show = 'off';
    plot_settings.gia = 'off';
end
ani_settings.gravitational_acceleration_vec_show ='on';
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Animation Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Link radius [m]
ani_settings.link_radius = 0.0185;
%%% Base thickness [m]
ani_settings.base_upper_thickness = 0.07;
ani_settings.base_lower_thickness = 0.03;
%%% Robot color
ani_settings.robot_base_upper_color = [0.1, 0.1, 0.1];
ani_settings.robot_base_lower_color = [0.1, 0.1, 0.1];
ani_settings.robot_limb_color       = [0.1, 0.1, 0.1];
%%% Robot transparency
ani_settings.robot_base_alpha = 0.5;
ani_settings.robot_limb_alpha = 0.5;

%%% Graspable points viz. on/off
ani_settings.graspable_points_show = 'on';
%%% Graspable points size
ani_settings.graspable_points_size = 40;
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

% ani_settings.camera_az=-90;
% ani_settings.camera_el = 90;
if ~strcmp(gait_planning_param.type,'crawl_uno_ver') && ~strcmp(gait_planning_param.type,'koizumi')
    ani_settings.graspable_points_show = 'off';
   ani_settings.graspable_points_in_reachable_area_show = 'off' ;
   ani_settings.reachable_area_show = 'off';
   ani_settings.goal_show = 'off';
   ani_settings.next_desired_position_show = 'off';
end
if strcmp(gait_planning_param.type,'koizumi')
    ani_settings.graspable_points_show = 'off';
    ani_settings.graspable_points_in_reachable_area_show = 'off' ;
    ani_settings.reachable_area_show = 'off';
    ani_settings.goal_show = 'off';
    ani_settings.next_desired_position_show = 'on';

    equilibrium_eval_param.expansion_factor = 0.01;
    ani_settings.z_vis_range = [-0.35 0.35];
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Save Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Save basic variables to csv file on/off
% save_settings.csv_file = 'on';
% 
% %%% Save TSM variables to csv file on/off
% save_settings.tsm = 'on';
% ani_settings.save = 'on';
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot footholds history on/off
plot_settings.footholds = 'on';
%%% Plot TSM
plot_settings.tsm = 'on';
if strcmp(equilibrium_eval_param.type,'tsm')
    plot_settings.gia_margin = 'off';
    plot_settings.gia_inclination_margin = 'off';
elseif strcmp(equilibrium_eval_param.type,'gia')
    plot_settings.tsm = 'off';
    plot_settings.gia_margin = 'on';
end

if strcmp(plot_settings.tsm,'on')
    save_settings.tsm = 'on';
end
if strcmp(plot_settings.gia,'on')
    save_settings.gia = 'on';
end
if strcmp(plot_settings.gia_margin,'on')
equilibrium_eval_param.type = 'gia';
save_settings.gia = 'on';
end

end