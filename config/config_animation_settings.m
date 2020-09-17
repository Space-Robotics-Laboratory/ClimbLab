%%%%%% Configuration
%%%%%% config_animation_settings
%%%%%% 
%%%%%% Configure default animation settings
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-08
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
ani_settings.base_thickness = 0.03;

%%% Robot color
ani_settings.robot_color = [0.2, 0.2, 0];
%%% Robot transparency
ani_settings.robot_alpha = 0.8;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Surface related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%% Next desired position vis. on/off
ani_settings.next_desired_position_show = 'off';
    %%% Next desired position line color  m : magenta
    ani_settings.next_desired_position_color = 'm';
    %%% Next desired position line width
    ani_settings.next_desired_position_line_width = 1;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Support triangle visualization
ani_settings.support_triangle_show = 'off';
    %%% Support triangle color
    ani_settings.support_triangle_color =[0 1.0 1.0];
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
    
end