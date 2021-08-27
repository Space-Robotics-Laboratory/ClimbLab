%%%%%% Configuration
%%%%%% config_gia_static
%%%%%% 
%%%%%% Define preset parameters
%%%%%% 
%%%%%% Created 2020-07-09
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-02-22
%%%%%% Kentaro Uno
%
%
% Load configurations for a template of GIA polyhedron in static case
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
    plot_settings] = config_gia_static_param(robot_param, environment_param, gait_planning_param, control_param, ...
    equilibrium_eval_param, ani_settings, save_settings, plot_settings)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Type of the robot to simulate ('HubRobo_v2_2_no_grip', 'HubRobo_v2_2_grip_to_palm', 'HubRobo_v3_1_grip_to_palm', 'HubRobo_v2_2_grip_to_spine')
robot_param.robot_type = 'HubRobo_grip_to_spine_old';
%%% x and y position of legs relative to base center [m]
robot_param.x_foot_dist = 0.12;
robot_param.y_foot_dist = 0.12;
%%% Height of base relative to map [m]
robot_param.base_height = 0.14;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Type of the surface ('flat_HR', 'rough', 'flat_003', 'flat_006', 'flat_008', 'flat_009', 'flat_012') 
environment_param.surface_type = 'flat_HR';
%%% Gravity [G]
environment_param.grav = 1;

%%% Dynamics on/off
environment_param.dynamics_flag = 'off';
%%% Maximum simulation time [s]
environment_param.time_max = 0;

%%% Surface Inclination [deg]
environment_param.inc = 70;

%%% Gripper detachment method ('none', 'max_holding_force')
environment_param.detachment_detection_method = 'none';

%%% Simulation stop if robot is stuck (selecting same grasping point)
environment_param.sim_stop_stuck = 'off';
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Gait type ('do_nothing', 'crawl_fixed_stride','crawl_gait_for_discrete_footholds')
gait_planning_param.type = 'do_nothing';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Control Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Controller type ('do_nothing', 'torque_PD')
control_param.type = 'do_nothing';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Equilibrium evaluation method ('none', 'tsm', 'gia', 'tsm_and_gia')
equilibrium_eval_param.type = 'gia';


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Animation Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% x-axis limits [m] [[-0.25, 0.4], [-0.25, .25], [0, .5]]
ani_settings.x_lim = [-0.27, 0.25];
%%% y-axis limits [m] [[-0.25, 0.4], [-0.25, .25]]
ani_settings.y_lim = [-0.4, 0.35];
%%% z-axis limits [m] [[-0.08, 0.1], [-0.1, .14]] [-0.25 0.25]
ani_settings.z_lim = [-0.3, 0.25];
%%% Lower z-axis limit same as lower point on surface on/off
ani_settings.z_lim_low_is_surface = 'off';
%%% 'on' camera follows robot, 'off' camera fixed
ani_settings.move_camera = 'off';
%%% Camera angle azimuth and elevation [deg]
ani_settings.camera_az = -160;
ani_settings.camera_el = 12;

%%% Link radius [m]
ani_settings.link_radius = 0.010;
%%% Base thickness [m]
ani_settings.base_upper_thickness = 0.03;
ani_settings.base_lower_thickness = 0.03;
%%% Robot color
ani_settings.robot_base_upper_color = [0.2, 0.2, 0];
ani_settings.robot_base_lower_color = [0.2, 0.2, 0];
ani_settings.robot_limb_color       = [0.2, 0.2, 0];
%%% Robot transparency
ani_settings.robot_base_alpha = 0.5;
ani_settings.robot_limb_alpha = 0.5;

%%% Transformation from acceleration to visualize GIA Stable Region, GIA
%%% vector in the position coordinate -> this scale is used for all 
%%% accelerational variables visualization
ani_settings.acceleration_expansion_factor = 0.02;

    %%% GIA Stable Region visualize on/off (requires equilibrium evaluation method as 'gia')
    ani_settings.gia_stable_region_show = 'on';
        %%% Stability polyhedron face color
        ani_settings.polyh_Face_color = [0 0 1];
        %%% Stability polyhedron face alpha
        ani_settings.polyh_Face_alpha = 0.25;
        %%% Stability polyhedron edge color
        ani_settings.polyh_Edge_color = [0 0 1];
        %%% Stability polyhedron edge width
        ani_settings.polyh_Edge_width = 2;

    %%% Gravitational Acceleration vector visualize on/off
    ani_settings.gravitational_acceleration_vec_show = 'on';
        %%% Gravity Acceleration vector color
        ani_settings.gravitational_acceleration_vec_color = [0 0.7 0];
        %%% Gravity Acceleration vector width
        ani_settings.gravitational_acceleration_vec_width = 3;

    %%% GIA vector visualize on/off
    ani_settings.gia_vec_show = 'on';
        %%% GIA vector color
        ani_settings.gia_vec_color = [1 0 0];
        %%% GIA vector width
        ani_settings.gia_vec_width = 3;

%%% Transformation from force to visualize Fg (gravitational force) and 
%%% Fe (reaction force) Stable Region in the position coordinate 
%%% -> this scale is used for all force-dimensional variables visualization
ani_settings.force_expansion_factor = 0.04;
    
    %%% Gravitational Force vector visualize on/off
    ani_settings.gravitational_force_vec_show = 'off';
        %%% Gravity vector color
        ani_settings.gravitational_force_vec_color = [0 0.7 0];
        %%% Gravity vector width
        ani_settings.gravitational_force_vec_width = 3;    
    
    %%% Reaction force vectors vis. on/off
    ani_settings.reaction_force_show = 'off';
        %%% Reaction force vectors color
        ani_settings.reaction_force_vec_color = [1 0 0];
        %%% Reaction force vectors color width
        ani_settings.reaction_force_vec_width = 3;    

%%% Trajectory visualization on/off
ani_settings.trajectory_show = 'off';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Save Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save_settings.csv_file = 'off';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot Base position
plot_settings.base_pos = 'off';

end