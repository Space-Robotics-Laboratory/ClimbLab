%%%%%% Configuration
%%%%%% config_default_param
%%%%%% 
%%%%%% Define default parameters
%%%%%% 
%%%%%% Created 2020-07-09
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-09-08
%
%
% Define configurations for default parameters. This file is not actually used, but it contains a list of ALL parameters. use
% it as a reference for the variables in your USER configuration.
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
    plot_settings] = config_default_param(robot_param, environment_param, gait_param, control_param, ...
    equilibrium_eval_param, ani_settings, save_settings, plot_settings)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Type of the robot to simulate ('HubRobo_v2_2_no_grip', 'HubRobo_v2_2_grip_to_palm', 'HubRobo_v3_1_grip_to_palm', 'HubRobo_v2_2_grip_to_spine')
robot_param.robot_type = 'HubRobo_v2_2_grip_to_palm';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Position settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% x and y position of legs relative to base center [m]
robot_param.foot_dist  = 0.12;
%%% Height of base relative to map [m]
robot_param.base_height = 0.12;
%%% Base position [m] 2x1 vector. or 'default' for default setting
robot_param.base_pos_xy = [0;0];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Type of the surface ('flat_HR', 'rough', 'flat_003', 'flat_006', 'flat_008', 'flat_009', 'flat_012') 
environment_param.surface_type = 'rough';
%%% Gravity [G]
environment_param.grav = 1/6;

%%% Dynamics on/off
environment_param.dynamics_flag = 'on';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Time settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Time-step [s]
environment_param.time_step = 0.001;
%%% Maximum simulation time [s]
environment_param.time_max = 8;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Surface settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Surface Inclination [deg]
environment_param.inc = 0;
%%% Ground reaction force stiffness coefficient
environment_param.surface_K = 1000;
%%% Ground reaction force damping coefficient (scalar)
environment_param.surface_D = 1;
%%% Graspable points detection type
environment_param.graspable_points_detection_type = 'all';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Gait type ('do_nothing', 'crawl_fixed_stride','crawl_uno_ver')
gait_param.type = 'crawl_fixed_stride';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Crawl gait settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Period of one cycle [s]
gait_param.T = 4;
%%% Duty cycle
gait_param.beta = 0.75;
%%% Horizontal distance per step (stride) [m]
gait_param.step_length = 0.05;
%%% Step height [m]
gait_param.step_height = 0.03;
%%% Walking sequence: 1 - 3 - 4 - 2
gait_param.sequence = [1; 3; 4; 2];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Uno gait settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Goal position [m]
gait_param.goal = [0.4;0;-0.08];


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Control Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Controller type ('do_nothing', 'torque_PD')
control_param.type = 'torque_PD';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PD controller settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Controller Proportional (P) gain
control_param.kp = 3.00;
%%% Controller Derivative (D) gain
control_param.kd = 0.02;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Equilibrium evaluation method ('none', 'tsm', 'gia')
equilibrium_eval_param.type = 'tsm';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GIA polyhedron settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plot polyhderon on animation on/off (requires equilibrium evaluation method as 'gia')
equilibrium_eval_param.plot_polyhedron = 'off';
% Transformation from acceleration to plot in the position coordinate
equilibrium_eval_param.expansion_factor = 0.02;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Animation Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Display animation on/off
ani_settings.display = 'on';
%%% Save animation
ani_settings.save = 'off';

%%% Frame rate [frames/s] [20,25,50]
ani_settings.frame_rate = 20;
%%% Animation video resolution [px] [[1280 720], [640 480]]
ani_settings.animation_resolution = [640 480];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Camera related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% x-axis limits [m] [[-0.25, 0.4], [-0.25, .25], [0, .5]]
ani_settings.x_lim = [-0.25, 0.4];
%%% y-axis limits [m] [[-0.25, 0.4], [-0.25, .25]]
ani_settings.y_lim = [-0.25, 0.25];
%%% z-axis limits [m] [[-0.08, 0.1], [-0.1, .14]] [-0.25 0.25]
ani_settings.z_lim = [-0.18, 0.15];
%%% Lower z-axis limit same as lower point on surface on/off
ani_settings.z_lim_low_is_surface = 'on';

%%% 'on' camera follows robot, 'off' camera fixed
ani_settings.move_camera = 'on';
    %%% x-axis visualization range [m] [[-0.25, 0.4], [-0.25, .25], [0, .5]]
    ani_settings.x_vis_range = ani_settings.x_lim - mean(ani_settings.x_lim);
    %%% y-axis visualization range [m] [[-0.25, 0.4], [-0.25, .25]]
    ani_settings.y_vis_range = ani_settings.y_lim - mean(ani_settings.y_lim);
    %%% z-axis visualization range [m] [[-0.08, 0.1], [-0.1, .14]]
    ani_settings.z_vis_range = ani_settings.z_lim - mean(ani_settings.z_lim);


%%% Camera angle azimuth and elevation [deg]
ani_settings.camera_az =-20;
ani_settings.camera_el = 12;

%%% Font name
ani_settings.font_name = 'Times New Roman';
%%% Font size
ani_settings.font_size = 25;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Link radius [m]
ani_settings.link_radius = 0.012;
%%% Base thickness [m]
ani_settings.base_thickness = 0.03;

%%% Robot color
ani_settings.robot_color = [0.2, 0.2, 0];
%%% Robot transparency
ani_settings.robot_alpha = 0.8;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Surface related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Surface viz. on/off
ani_settings.surface_show = 'on';
    %%% Surface grid color
    ani_settings.grid_color = [0.9 0.9 0.9];
    %%% Surface color
    ani_settings.surface_color = gray;

%%% Graspable points viz. on/off
ani_settings.graspable_points_show = 'off';
    %%% Graspable points color
    ani_settings.graspable_points_color = [0, 0, 1.0];
    %%% Graspable points marker
    ani_settings.graspable_points_marker = '.';
    %%% Graspable points size
    ani_settings.graspable_points_size = 5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Graspable points in reachable area viz. on/off
ani_settings.graspable_points_in_reachable_area_show = 'off';
    %%% Graspable points in reachable area color
    ani_settings.graspable_points_in_reachable_area_color = [0, 1.0, 0];
    %%% Graspable points in reachable area marker
    ani_settings.graspable_points_in_reachable_area_marker = '.';
    %%% Graspable points in reachable area size
    ani_settings.graspable_points_in_reachable_area_size = 20;

%%% Reachable area viz. on/off
ani_settings.reachable_area_show = 'off';
    %%% Reachable area line color
    ani_settings.reachable_area_line_color = 'magenta';
    %%% Reachable area line width
    ani_settings.reachable_area_line_width = 1;

%%% Goal vis. on/off
ani_settings.goal_show = 'off';
    %%% Goal color
    ani_settings.goal_color = [0 0 1.0];
    %%% Goal marker
    ani_settings.goal_marker = '.';
    %%% Goal size
    ani_settings.goal_size = 25;
    %%% Goal vis. height from the ground
    ani_settings.goal_vis_height = 0.0;

%%% Next desired position vis. on/off
ani_settings.next_desired_position_show = 'off';
    %%% Next desired position line color  m : magenta
    ani_settings.next_desired_position_color = 'm';
    %%% Next desired position line width
    ani_settings.next_desired_position_line_width = 1;
    
%%% Plot trajectory
ani_settings.trajectory_show = 'on';
	%%% Trajectory line color
    ani_settings.trajectory_color = 'c';
	%%% Trajectory line widith
    ani_settings.trajectory_width = 3;
	%%% Trajectory line type
    ani_settings.trajectory_line_type = ':';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Support triangle visualization
ani_settings.support_triangle_show = 'off';
    %%% Support triangle color
    ani_settings.support_triangle_color = [0 1.0 1.0];
    %%% Support triangle edge line color
    ani_settings.support_triangle_edge_color = [0.0 0.0 0.0];
    %%% Support triangle transparency
    ani_settings.support_triangle_alpha = 0.5;

%%% Stability polyhedron face color
ani_settings.polyh_Face_color = [0 0 1];
%%% Stability polyhedron face alpha
ani_settings.polyh_Face_alpha = 0.25;
%%% Stability polyhedron edge color
ani_settings.polyh_Edge_color = [0 0 1];
%%% Stability polyhedron edge width
ani_settings.polyh_Edge_width = 2;

%%% Gravity vector visualize on/off
ani_settings.gravity_vec_show = 'off';
    %%% Gravity vector color
    ani_settings.gravity_vec_color = [0 0.7 0];
    %%% Gravity vector width
    ani_settings.gravity_vec_width = 3;

%%% GIA vector visualize on/off
ani_settings.gia_vec_show = 'off';
    %%% GIA vector color
    ani_settings.gia_vec_color = [1 0 0];
    %%% GIA vector width
    ani_settings.gia_vec_width = 3;

%%% CoM projection point vis. on/off
ani_settings.com_projection_show = 'off';
    %%% CoM projection point color
    ani_settings.com_projection_color = [1 0 0];
    %%% CoM projection point marker
    ani_settings.com_projection_marker = '.';
    %%% CoM projection point size
    ani_settings.com_projection_size = 25;
    %%% CoM projection point vis. height from the ground
    ani_settings.com_projection_vis_height = 0.0;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Save Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Save basic variables to csv file on/off
save_settings.csv_file = 'on';

%%% Save TSM variables to csv file on/off
save_settings.tsm = 'off';
%%% Save variables to csv file on/off
save_settings.gia = 'off';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Font name
plot_settings.font_name = 'Times New Roman';
%%% Font size
plot_settings.font_size = 25;
%%% Line width
plot_settings.width = 3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Basic graphs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 2 ~ 100
%%% Plot Base position
plot_settings.base_pos = 'on';
plot_settings.base_pos_fig_number = 2;
%%% Plot Base orientation
plot_settings.base_ori = 'off';
plot_settings.base_ori_fig_number = 3;
%%% Plot Base velocity
plot_settings.base_vel = 'off';
plot_settings.base_vel_fig_number = 4;
%%% Plot Base angular velocity
plot_settings.base_angvel = 'off';
plot_settings.base_angvel_fig_number = 5;
%%% Plot Base velocity
plot_settings.base_acc = 'off';
plot_settings.base_acc_fig_number = 6;
%%% Plot Base angular velocity
plot_settings.base_angacc = 'off';
plot_settings.base_angacc_fig_number = 7;

%%% Plot Joint angular position
plot_settings.joint_pos = 'off';
plot_settings.joint_pos_fig_number = 8;
%%% Plot Joint angular velocity
plot_settings.joint_vel = 'off';
plot_settings.joint_vel_fig_number = 9;
%%% Plot Joint angular acceleration
plot_settings.joint_acc = 'off';
plot_settings.joint_acc_fig_number = 10;
%%% Plot Joint torque
plot_settings.joint_torque = 'off';
plot_settings.joint_torque_fig_number = 11;

%%% Plot Leg Position
plot_settings.leg_pos = 'off';
plot_settings.leg_pos_fig_number = 12; % and 13, 14, 15 are also used 

%%% Plot Reaction Force
plot_settings.reaction_force = 'off';
plot_settings.reaction_force_fig_number = 16; % and 17~23  are also used 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 101 ~ 200
%%% Plot footholds history on/off
plot_settings.footholds = 'off';
plot_settings.footholds_fig_number = 101;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 201 ~ 300
%%% Plot TSM
plot_settings.tsm = 'off';
plot_settings.tsm_fig_number = 201;

%%% Plot Acceleration margin
plot_settings.acc_margin = 'off';
plot_settings.acc_margin_fig_number = 202;

%%% Plot Inclination margin
plot_settings.inclination_margin = 'off';
plot_settings.inclination_margin_fig_number = 203;

end