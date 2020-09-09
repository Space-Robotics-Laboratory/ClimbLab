%%%%%% Visualize
%%%%%% vis_graspable_points
%%%%%%
%%%%%% Disp graspable points
%%%%%%
%%%%%% Created 2020-05-14
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2020-05-14
%
%
% Disp graspable points
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         surface_param                : Position of Graspable points
%         inc                          : Surface inclination [deg] (scalar)
%         ani_settings.graspable_color : Color for points [RGB] (1x3 vector)
%         ani_settings.graspable_marker: Type of marker (String)
%         ani_settings.graspable_size  : Size of points (scalar)


function vis_graspable_points(surface_param, inc, ani_settings)
if strcmp(ani_settings.graspable_points_show,'on')
    rot = rpy2dc([0;pi*inc/180;0])';
    graspable_points = rot' * surface_param.graspable_points;
    plot3(graspable_points(1,:),graspable_points(2,:),graspable_points(3,:),ani_settings.graspable_points_marker,'Color',ani_settings.graspable_points_color,'MarkerSize',ani_settings.graspable_points_size);
end
end