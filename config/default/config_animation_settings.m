%%%%%% Configuration
%%%%%% config_animation_settings
%%%%%% 
%%%%%% Configure default animation settings
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-07-06
%%%%%% Keigo Haji
%
%
% Load default configurations for saving
%
% Function variables:
%
%     OUTPUT
%         ani_settings
%     INPUT
%         -

function ani_settings = config_animation_settings()
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Display animation on/off
ani_settings.display = 'on';
%%% Save animation
ani_settings.save = 'off';
%%% elapsed time show on/off
ani_settings.animation_elapsed_time_show = 'on';

%%% Frame rate [frames/s] [20,25,50]
ani_settings.frame_rate = 20;
%%% Animation video resolution [px] [[1280 720], [640 480]]
ani_settings.animation_resolution = [640 480];


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Camera related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Link radius [m]
ani_settings.link_radius = 0.012;
%%% Base thickness [m]
ani_settings.base_upper_thickness = 0.03;
ani_settings.base_lower_thickness = 0.03;
%%% Base horisontal scaling factor
ani_settings.base_xy_scale_factor = 1.0;
%%% Robot color
ani_settings.robot_base_upper_color = [0.2, 0.2, 0];
ani_settings.robot_base_lower_color = [0.2, 0.2, 0];
ani_settings.robot_limb_color       = [0.2, 0.2, 0];
%%% Robot transparency
ani_settings.robot_base_alpha = 0.8;
ani_settings.robot_limb_alpha = 0.8;

%%% Frames at CoM and each joint on/off
ani_settings.frames_show = 'off';
    ani_settings.frames_line_width = 2.0;
    ani_settings.frames_size  = 0.05;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Surface related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    %%% Graspable points alpha
    ani_settings.graspable_points_alpha = 1.0;
    %%% Graspable points marker
    ani_settings.graspable_points_marker = 'o';
    %%% Graspable points size
    ani_settings.graspable_points_size = 10;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
    
%%% Plot Limb trajectory
ani_settings.trajectory_show = 'on';
	%%% Trajectory line color
    ani_settings.trajectory_color = 'c';
	%%% Trajectory line widith
    ani_settings.trajectory_width = 3;
	%%% Trajectory line type
    ani_settings.trajectory_line_type = ':';
    
%%% Plot CoM trajectory
ani_settings.com_proj_trajectory_show = 'off';
	%%% Trajectory line color
    ani_settings.com_proj_trajectory_color = 'k';
	%%% Trajectory line widith
    ani_settings.com_proj_trajectory_width = 2;
	%%% Trajectory line type
    ani_settings.com_proj_trajectory_line_type = '-';    
        
    
    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Support triangle visualization
ani_settings.support_triangle_show = 'off';
%%% Support triangle above the robot visualization
    ani_settings.robot_top_support_triangle_show = 'off';
    %%% Support triangle color
    ani_settings.support_triangle_color = [0 136/255 170/255];
    %%% Support triangle edge line color
    ani_settings.support_triangle_edge_color = [0.0 0.0 0.0];
    %%% Support triangle transparency
    ani_settings.support_triangle_alpha = 0.5;

%%% Transformation from acceleration to visualize GIA Stable Region, GIA
%%% vector in the position coordinate -> this scale is used for all 
%%% accelerational variables visualization
ani_settings.acceleration_expansion_factor = 0.02;

    %%% GIA Stable Region visualize on/off (requires equilibrium evaluation method as 'gia')
    ani_settings.gia_stable_region_show = 'off';
        %%% GIA Stable Region face color
        ani_settings.gia_stable_region_Face_color = [0 0 1];
        %%% GIA Stable Region face alpha
        ani_settings.gia_stable_region_Face_alpha = 0.25;
        %%% GIA Stable Region edge color
        ani_settings.gia_stable_region_Edge_color = [0 0 1];
        %%% GIA Stable Region edge width
        ani_settings.gia_stable_region_Edge_width = 2;

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

%%% CoM projection point vis. on/off
ani_settings.com_projection_show = 'off';
%%% CoM projection point above the robot vis. on/off
ani_settings.robot_top_com_projection_show = 'off';
    %%% CoM projection point color
    ani_settings.com_projection_color = [1 0 0];
    %%% CoM projection point marker
    ani_settings.com_projection_marker = '.';
    %%% CoM projection point size
    ani_settings.com_projection_size = 25;
    %%% CoM projection point vis. height from the ground
    ani_settings.com_projection_vis_height = 0.0;


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
    ani_settings.sensing_camera_fov_LineColor = 'k';
%%% RealSense Camera fov filling vis. on/off
ani_settings.sensing_fov_face_filling = 'off';
    %%% Filling color
    ani_settings.sensing_fov_face_color = 'c';
    %%% Filling color alpha
    ani_settings.sensing_fov_face_alpha = 0.5;  
    
%%% Sensed Graspable points viz. on/off
ani_settings.sensed_graspable_points_show = 'off';
    %%% Sensed Graspable points color
    ani_settings.sensed_graspable_points_color = [1.0, 0, 1.0];
    %%% Sensed Graspable points alpha
    ani_settings.sensed_graspable_points_alpha = 1.0;
    %%% Sensed Graspable points marker
    ani_settings.sensed_graspable_points_marker = 'o';
    %%% Sensed Graspable points size
    ani_settings.sensed_graspable_points_size = 10;

end