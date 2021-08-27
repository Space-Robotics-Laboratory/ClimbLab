%%%%%% Visualize
%%%%%% vis_sensed_graspable_points
%%%%%%
%%%%%% Display sensed graspable points
%%%%%%
%%%%%% Created 2021-02-15
%%%%%% Keigo Haji
%%%%%% Last update: 2021-04-05
%%%%%% Kentaro Uno
%
%
% Display graspable points sensed by the RealSense Camera
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         surface_param.sensed_graspable_points : Position of Sensed Graspable points
%         inc                                   : Surface inclination [deg] (scalar)
%         ani_settings.sensed_graspable_color   : Color for points [RGB] (1x3 vector)
%         ani_settings.sensed_graspable_alpha   : Alpha for marker (0 -- 1)
%         ani_settings.sensed_graspable_marker  : Type of marker (String)
%         ani_settings.sensed_graspable_size    : Size of points (scalar)


function vis_sensed_graspable_points(surface_param, inc, ani_settings)

if strcmp(ani_settings.sensed_graspable_points_show,'on')
    % Rotation matrix
    rot = rpy2dc([0;pi*inc/180;0])';
    
    % Transform coordinates from the ground coordinate system to the
    % inertial coordinate system. 
    sensed_graspable_points = rot' * surface_param.sensed_graspable_points;
    
   % Display
    scatter3(sensed_graspable_points(1,:),sensed_graspable_points(2,:),sensed_graspable_points(3,:), ...
        ani_settings.sensed_graspable_points_size, ...
        ani_settings.sensed_graspable_points_marker, ...
        'MarkerEdgeColor', 'none', ...
        'MarkerFaceColor',ani_settings.sensed_graspable_points_color, ...
        'MarkerFaceAlpha',ani_settings.sensed_graspable_points_alpha);
end

end

