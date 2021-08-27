%%%%%% Visualize
%%%%%% vis_com_projection
%%%%%%
%%%%%% Display CoM projection point
%%%%%%
%%%%%% Created 2020-05-28
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2021-05-17
%%%%%% Kentaro Uno
%
%
% Display Center of Mass projection
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param.com_proj_history    : com projection history 
%     INPUT
%		  LP                                      : Link parameters (SpaceDyn class)
%         SV                                      : State Variables
%         inc                                     : Surface inclination [deg] (scalar)
%         ani_settings.com_projection_color       : Color for points [RGB] (1x3 vector)
%         ani_settings.com_projection_marker      : Type of marker (String)
%         ani_settings.com_projection_size        : Size of points (scalar)
%         ani_settings.com_projection_vis_height  : visualization height from the ground
%                                                   to see it well (scalar)
%         ani_settings.com_projection_show        : on / off


function gait_planning_param = vis_com_projection(LP, SV, inc, ani_settings, gait_planning_param)

CoM = upd_CoM(LP, SV);

if strcmp(ani_settings.com_projection_show,'on')
    [a,b,c] =  get_map_pos(CoM(1),CoM(2));
    c = c + ani_settings.com_projection_vis_height;
    com_proj = rpy2dc([0;pi*inc/180;0])*[CoM(1)-norm(CoM(3)-c)*tan(inc*pi/180);CoM(2);c];
    plot3(com_proj(1,1),com_proj(2,1),com_proj(3,1),ani_settings.com_projection_marker,'Color',ani_settings.com_projection_color,'MarkerSize',ani_settings.com_projection_size);
end

if strcmp(ani_settings.robot_top_com_projection_show,'on')
    [a,b,c] =  get_map_pos(CoM(1),CoM(2));
    c = c + ani_settings.com_projection_vis_height;
    com_proj = rpy2dc([0;pi*inc/180;0])*[CoM(1)-norm(CoM(3)-c)*tan(inc*pi/180);CoM(2);c];
    base_position_in_inertial_frame = rpy2dc([0;pi*inc/180;0]) * SV.R0;
    plot3(com_proj(1,1),com_proj(2,1),base_position_in_inertial_frame(3)+2.0*CoM(3), ani_settings.com_projection_marker,'Color',ani_settings.com_projection_color,'MarkerSize',ani_settings.com_projection_size);
end

    % log the history path of the CoM projection 
    com_proj_hist = rpy2dc([0;pi*inc/180;0])*[CoM(1)-norm(CoM(3))*tan(inc*pi/180);CoM(2);0];
    gait_planning_param.com_projection_history = [gait_planning_param.com_projection_history com_proj_hist];
end