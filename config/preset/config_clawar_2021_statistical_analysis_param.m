%%%%%% Configuration
%%%%%% config_clawar_2021_statistical_analysis_param
%%%%%% 
%%%%%% Define preset parameters
%%%%%% 
%%%%%% --------------------------------------------------------------------
%%%%%% This configration file can reproduce the simulation result used in
%%%%%% CLAWAR 2021 proceedings paper by K. Uno and G. Valsecchi et al.
%%%%%% Proceedings Paper URL: 
%%%%%% https://******** (to be added)
%%%%%% --------------------------------------------------------------------
%%%%%% 
%%%%%% Created 2021-02-16
%%%%%% by Kentaro Uno
%%%%%% Last update: 2021-08-23
%%%%%% by Kentaro Uno
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
    plot_settings] = config_clawar_2021_statistical_analysis_param(robot_param, environment_param, gait_planning_param, control_param, ...
    equilibrium_eval_param, ani_settings, save_settings, plot_settings)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Type of the robot to simulate ('HubRobo_v2_2_no_grip', 'HubRobo_v2_2_grip_to_palm', 'HubRobo_v3_1_grip_to_palm', 'HubRobo_v2_2_grip_to_spine')
robot_param.robot_type = 'Climbing_ANYmal';

% Nomally, the following four items should be defined in the LP file, 
% but it is also possible to specify the joint allocation type and offset 
% angles (theta1 and 2) for the hip joints here for more convenience.

% if you set "insect" for the following type, theta1 and 2 are forced to be pi/4 and -pi/2 [rad],
% respectively.
robot_param.joint_allocation_type = 'insect'; % this can also be set in ini_LP file
% for the mammalian robot, you can specify the leg configuration from "oo", "xx", and "M".
robot_param.leg_config_type = 'oo'; % this can also be set in LP file
robot_param.theta_1 = 0.1440; % [rad]
% note that if you put the negative num for theta_2, the IK solver take automatically
% 'oo' configuration solution
robot_param.theta_2 = -0.1955; % [rad]

%%% "Initial" x and y position of legs relative to base center [m]
horizontal_length_from_base_corner = 0.45; % [m]
yaw_angle = 45; % [deg]
base_width = 0.3; % [m]
base_length = 0.3; % [m]
robot_param.x_foot_dist = base_length/2 + horizontal_length_from_base_corner * cos(yaw_angle*pi/180);
robot_param.y_foot_dist = base_width/2 + horizontal_length_from_base_corner * sin(yaw_angle*pi/180);
% robot_param.x_foot_dist = 0.4;
% robot_param.y_foot_dist = 0.4;

%%% Height of base relative to map [m]
robot_param.base_height = 0.2;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Type of the surface ('flat_HR', 'flat_HR_5m_x_5m', 'rough', 'flat_003', 'flat_006', 'flat_008', 'flat_009', 'flat_012') 
environment_param.surface_type = 'flat_HR_5m_x_5m';
%%% Gravity [G] (1/6: moon, 3/8: mars, 10^-5: asteroid)
environment_param.grav = 1; 

%%% Dynamics on/off
environment_param.dynamics_flag = 'on';
%%% Maximum simulation time [s]
environment_param.time_max = 2;

%%% Surface Inclination [deg]
environment_param.inc = 45;

% Floor reaction force stiffness coefficient
environment_param.surface_K = 100000;
% Floor reaction force damping coefficient
environment_param.surface_D = 100.0;  

%%% Gripper detachment method ('none', 'max_holding_force')
environment_param.detachment_detection_method = 'none';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Gait type ('do_nothing', 'crawl_fixed_stride','crawl_uno_ver')
gait_planning_param.type = 'do_nothing';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Control Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Controller type ('do_nothing', 'torque_PD')
control_param.type = 'torque_PD';
%%% Controller Proportional (P) gain
control_param.kp = 500.0;
%%% Controller Derivative (D) gain
control_param.kd = 10.0;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Equilibrium evaluation method ('none', 'tsm', 'gia', 'tsm_and_gia')
equilibrium_eval_param.type = 'tsm_and_gia';

% Plot polyhderon on animation on/off (requires equilibrium evaluation method as 'gia')
equilibrium_eval_param.plot_polyhedron = 'off';
% Transformation from acceleration to plot in the position coordinate
equilibrium_eval_param.expansion_factor = 0.02;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Animation Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% x-axis limits [m] [[-0.25, 0.4], [-0.25, .25], [0, .5]]
ani_settings.x_lim = [-0.8, 0.8];
%%% y-axis limits [m] [[-0.25, 0.4], [-0.25, .25]]
ani_settings.y_lim = [-0.9, 0.9];
%%% z-axis limits [m] [[-0.08, 0.1], [-0.1, .14]] [-0.25 0.25]
ani_settings.z_lim = [-0.6, 0.8];
%%% Lower z-axis limit same as lower point on surface on/off
ani_settings.z_lim_low_is_surface = 'off';
%%% 'on' camera follows robot, 'off' camera fixed
ani_settings.move_camera = 'off';
%%% Camera angle azimuth and elevation [deg]
ani_settings.camera_az =-0;
ani_settings.camera_el = 0;

%%% Font name
ani_settings.font_name = 'Times New Roman';
%%% Font size
ani_settings.font_size = 25;

%%% Link radius [m]
ani_settings.link_radius = 0.010;
%%% Base thickness [m]
ani_settings.base_upper_thickness = 0.03;
ani_settings.base_lower_thickness = 0.03;
%%% Robot color
ani_settings.robot_base_upper_color = [0.2, 0.2, 0.2];
ani_settings.robot_base_lower_color = [0.2, 0.2, 0.2];
ani_settings.robot_limb_color       = [0.2, 0.2, 0.2];
%%% Robot transparency
ani_settings.robot_base_alpha = 1.0;
ani_settings.robot_limb_alpha = 1.0;

%%% Stability polyhedron face color
ani_settings.polyh_Face_color = [0 0 1];
%%% Stability polyhedron face alpha
ani_settings.polyh_Face_alpha = 0.25;
%%% Stability polyhedron edge color
ani_settings.polyh_Edge_color = [0 0 1];
%%% Stability polyhedron edge width
ani_settings.polyh_Edge_width = 2;

%%% Gravity vector visualize on/off
ani_settings.gravity_vec_show = 'on';
    %%% Gravity vector color
    ani_settings.gravity_vec_color = [0 0.7 0];
    %%% Gravity vector width
    ani_settings.gravity_vec_width = 3;

%%% GIA vector visualize on/off
ani_settings.gia_vec_show = 'on';
    %%% GIA vector color
    ani_settings.gia_vec_color = [1 0 0];
    %%% GIA vector width
    ani_settings.gia_vec_width = 3;
    
ani_settings.trajectory_show = 'off';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Save Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Save basic variables to csv file on/off
save_settings.csv_file = 'on';
%%% Save TSM variables to csv file on/off (requires equilibrium evaluation method as 'tsm')
save_settings.tsm = 'on';
%%% Save GIA variables to csv file on/off (requires equilibrium evaluation method as 'gia')
save_settings.gia = 'on';
%%% Save manipularbility measure to csv file on/off 
save_settings.manipulability = 'on';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
plot_settings.manipulability = 'on';
end