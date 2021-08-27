%%%%%% Configuration
%%%%%% config_anymal_climb_test
%%%%%% 
%%%%%% Define preset parameters
%%%%%% 
%%%%%% Created 2021-01-18
%%%%%% Kentaro Uno
%%%%%% Last update: 2021-02-22
%%%%%% Kentaro Uno
%
%
% Load configurations for example of ANYmal robot climbing
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
    plot_settings] = config_anymal_climb_test_param(robot_param, environment_param, gait_planning_param, control_param, ...
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
robot_param.leg_config_type = 'xx'; % this can also be set in LP file
robot_param.theta_1 = 0.1440; % [rad]
% note that if you put the negative num for theta_2, the IK solver take automatically
% 'oo' configuration solution
robot_param.theta_2 = -0.1955; % [rad]

%%% "Initial" x and y position of legs relative to base center [m]
robot_param.x_foot_dist = 0.6;
robot_param.y_foot_dist = 0.3;
%%% Height of base relative to map [m]
robot_param.base_height = 0.2;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Type of the surface ('flat_HR', 'flat_HR_5m_x_5m', 'rough', 'flat_003', 'flat_006', 'flat_008', 'flat_009', 'flat_012') 
environment_param.surface_type = 'flat_HR_5m_x_5m';
%%% Gravity [G]
environment_param.grav = 1;

%%% Dynamics on/off
environment_param.dynamics_flag = 'on';
%%% Maximum simulation time [s]
environment_param.time_max = 2;

%%% Surface Inclination [deg]
environment_param.inc = 0;

% Floor reaction force stiffness coefficient
environment_param.surface_K = 100000;
% Floor reaction force damping coefficient
environment_param.surface_D = 100.0;  

%%% Gripper detachment method ('none', 'max_holding_force')
environment_param.detachment_detection_method = 'max_holding_force';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Gait type ('do_nothing', 'crawl_fixed_stride','crawl_gait_for_discrete_footholds','nonperiodic_gait_for_discrete_footholds' )
gait_planning_param.type = 'crawl_fixed_stride';

%%% Horizontal distance per step (stride) [m]
gait_planning_param.step_length = 0.1;
%%% Step height [m]
gait_planning_param.step_height = 0.1;
%%% Period of one cycle [s]
gait_planning_param.T = 8;
%%% Allowable maximum stride [m]
gait_planning_param.allowable_max_stride = 0.2; 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Control Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PD controller settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Controller Proportional (P) gain
control_param.kp = 2000.0;
%%% Controller Derivative (D) gain
control_param.kd = 4.5;


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Equilibrium evaluation method ('none', 'tsm', 'gia', 'tsm_and_gia')
equilibrium_eval_param.type = 'tsm_and_gia';


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Animation Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% x-axis limits [m] [[-0.25, 0.4], [-0.25, .25], [0, .5]]
ani_settings.x_lim = [-1.0, 1.0];
%%% y-axis limits [m] [[-0.25, 0.4], [-0.25, .25]]
ani_settings.y_lim = [-1.0, 1.0];
%%% z-axis limits [m] [[-0.08, 0.1], [-0.1, .14]] [-0.25 0.25]
ani_settings.z_lim = [-0.3, 1.0];
%%% Lower z-axis limit same as lower point on surface on/off
ani_settings.z_lim_low_is_surface = 'off';
%%% 'on' camera follows robot, 'off' camera fixed
ani_settings.move_camera = 'off';
%%% Camera angle azimuth and elevation [deg]
ani_settings.camera_az =-20;
ani_settings.camera_el = 15;

%%% Font name
ani_settings.font_name = 'Times New Roman';
%%% Font size
ani_settings.font_size = 25;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Link radius [m]
ani_settings.link_radius = 0.030;
%%% Base thickness [m]
ani_settings.base_upper_thickness = 0.20;
ani_settings.base_lower_thickness = 0.05;
%%% Base horisontal scaling factor
ani_settings.base_xy_scale_factor = 1.0;
%%% Robot color
ani_settings.robot_base_upper_color = [0.3, 0.3, 0.3];
ani_settings.robot_base_lower_color = [0.0, 0.0, 0.0];
ani_settings.robot_limb_color = [0.3, 0.3, 0.3];
%%% Robot transparency
ani_settings.robot_base_alpha = 0.5;
ani_settings.robot_limb_alpha = 0.5;

%%% Frames at each joint on/off
ani_settings.frames_show = 'on';
    ani_settings.frames_line_width = 2.0;
    ani_settings.frames_size = 0.1;

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
    ani_settings.gravitational_acceleration_vec_show = 'off';
        %%% Gravity Acceleration vector color
        ani_settings.gravitational_acceleration_vec_color = [0 0.7 0];
        %%% Gravity Acceleration vector width
        ani_settings.gravitational_acceleration_vec_width = 3;

    %%% GIA vector visualize on/off
    ani_settings.gia_vec_show = 'off';
        %%% GIA vector color
        ani_settings.gia_vec_color = [1 0 0];
        %%% GIA vector width
        ani_settings.gia_vec_width = 3;

%%% Transformation from force to visualize Fg (gravitational force) and 
%%% Fe (reaction force) Stable Region in the position coordinate 
%%% -> this scale is used for all force-dimensional variables visualization
ani_settings.force_expansion_factor = 0.005;
    
    %%% Gravitational Force vector visualize on/off
    ani_settings.gravitational_force_vec_show = 'on';
        %%% Gravity vector color
        ani_settings.gravitational_force_vec_color = [0 0.7 0];
        %%% Gravity vector width
        ani_settings.gravitational_force_vec_width = 3;    
    
    %%% Reaction force vectors vis. on/off
    ani_settings.reaction_force_show = 'on';
        %%% Reaction force vectors color
        ani_settings.reaction_force_vec_color = [1 0 0];
        %%% Reaction force vectors color width
        ani_settings.reaction_force_vec_width = 2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Plot trajectory
ani_settings.trajectory_show = 'off';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Save Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Save basic variables to csv file on/off
save_settings.csv_file = 'on';
%%% Save TSM variables to csv file on/off
save_settings.tsm = 'on';
% %%% Save variables to csv file on/off
save_settings.gia = 'on';
%%% Save manipulability to csv file on/off
save_settings.manipulability = 'on';
%%% Save Maximum of absolute torque of all joint
save_settings.joint_max_torque = 'on';
%%% Save Root Mean Square (RMS of torque of all joint (*need Signal Processing Toolbox)
save_settings.joint_rms_torque = 'on';


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot Base position
plot_settings.base_pos = 'off';
%%% Plot footholds history on/off
plot_settings.footholds = 'on';
%%% Plot TSM
plot_settings.tsm = 'on';
%%% Plot Gravito-Inertial Acceleration margin
plot_settings.gia_margin = 'on';
%%% Plot manipulability
plot_settings.manipulability = 'on';
%%% Plot max torque of all joint
plot_settings.joint_max_torque = 'on';
%%% Plot RMS torque of all joint
plot_settings.joint_rms_torque = 'on';

end