%%%%%% Visualize
%%%%%% vis_graspable_points_in_reachable_area
%%%%%%
%%%%%% Display graspable points in reachable area
%%%%%%
%%%%%% Created 2020-05-14
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2020-05-14
%
%
% Display graspable points in reachable area
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         path_planning_param                                   : Parameters for Path Plannning (class)
%         inc                                                   : Surface inclination [deg] (scalar)
%         ani_settings.graspable_points_in_reachable_area_color : Color for points [RGB] (1x3 vector)
%         ani_settings.graspable_points_in_reachable_area_marker: Type of marker (String)
%         ani_settings.graspable_points_in_reachable_area_size  : Size of points (scalar)


function vis_graspable_points_in_reachable_area(path_planning_param, inc, ani_settings)
if strcmp(ani_settings.graspable_points_in_reachable_area_show,'on')
    rot = rpy2dc([0;pi*inc/180;0])';
    if ~isempty(path_planning_param.graspable_points_in_reachable_area)
        graspable_points_in_reachable_area= rot'*path_planning_param.graspable_points_in_reachable_area;
        plot3(graspable_points_in_reachable_area(1,:),graspable_points_in_reachable_area(2,:),graspable_points_in_reachable_area(3,:),ani_settings.graspable_points_in_reachable_area_marker,'Color',ani_settings.graspable_points_in_reachable_area_color,'MarkerSize',ani_settings.graspable_points_in_reachable_area_size)
    end
end
end