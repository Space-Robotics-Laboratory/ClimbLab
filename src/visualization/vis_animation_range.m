%%%%%% Visualize
%%%%%% vis_animation_range
%%%%%% Update visualization range
%%%%%%
%%%%%% Created 2020-07-07
%%%%%% Koizumi Yusuke
%%%%%% Last update: 2020-07-08
%
%
% Update visualization range
%
% Function variables:
%
%     OUTPUT
%         ani_settings.i_lim                       : Limits for the i-axis(1x2vector), i : x, y, z
%     INPUT
%         motion_planning_param                    : Parameters for motion plan (struct)
%         surface_param                            : Parameters for surface (struct)
%         ani_settings.i_lim                       : Limits for the i-axis(1x2vector), i : x, y, z
%         ani_settings.i_vis_range                 : Range for the i-axis (1x2 vector), i : x, y, z
%         inc                                      : Surface inclination [deg] (scalar)


function [ ani_settings ] = vis_animation_range(motion_planning_param, surface_param, ani_settings, inc)
if strcmp(ani_settings.move_camera,'on')
    ani_settings.x_lim = motion_planning_param.des_R0(1) + ani_settings.x_vis_range;
    ani_settings.y_lim = motion_planning_param.des_R0(2) + ani_settings.y_vis_range;
    ani_settings.z_lim = motion_planning_param.des_R0(3) + ani_settings.z_vis_range;

    if inc == 0 && strcmp(ani_settings.z_lim_low_is_surface,'on')
        ani_settings.z_lim(1) = surface_param.min;
    end
end
end