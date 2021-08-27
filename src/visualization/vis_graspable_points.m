%%%%%% Visualize
%%%%%% vis_graspable_points
%%%%%%
%%%%%% Display graspable points
%%%%%%
%%%%%% Created 2020-05-14
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2021-04-08
%%%%%% Keigo Haji
%
%
% Display graspable points
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         surface_param                : Position of Graspable points
%         inc                          : Surface inclination [deg] (scalar)
%         ani_settings.graspable_color : Color for points [RGB] (1x3 vector)
%         ani_settings.graspable_alpha : Alpha for marker (0 -- 1)
%         ani_settings.graspable_marker: Type of marker (String)
%         ani_settings.graspable_size  : Size of points (scalar)


function vis_graspable_points(surface_param, inc, ani_settings)
if strcmp(ani_settings.graspable_points_show,'on')
    % Rotation matrix
    rot = rpy2dc([0;pi*inc/180;0])';
    
    % Transform coordinates from the ground coordinate system to the
    % inertial coordinate system. 
    graspable_points = rot' * surface_param.graspable_points;
    

    % Display
    scatter3(graspable_points(1,:),graspable_points(2,:),graspable_points(3,:), ...
        ani_settings.graspable_points_size, ...
        ani_settings.graspable_points_marker, ...
        'MarkerEdgeColor','none', ...
        'MarkerFaceColor',ani_settings.graspable_points_color, ...
        'MarkerFaceAlpha',ani_settings.graspable_points_alpha);
end
end