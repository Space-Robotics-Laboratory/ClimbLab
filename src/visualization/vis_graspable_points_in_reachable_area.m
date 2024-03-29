%%%%%% Visualize
%%%%%% vis_graspable_points_in_reachable_area
%%%%%%
%%%%%% Display graspable points in reachable area
%%%%%%
%%%%%% Created 2020-05-14
%%%%%% Yusuke Koizumi
%%%%%% Last updated: 2020-11-18
%%%%%% Kentaro Uno
%
%
% Display graspable points in reachable area
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         gait_planning_param                                   : Parameters for Gait Plannning (class)
%         inc                                                   : Surface inclination [deg] (scalar)
%         ani_settings.graspable_points_in_reachable_area_color : Color for points [RGB] (1x3 vector)
%         ani_settings.graspable_points_in_reachable_area_marker: Type of marker (String)
%         ani_settings.graspable_points_in_reachable_area_size  : Size of points (scalar)


function vis_graspable_points_in_reachable_area(gait_planning_param, inc, ani_settings)
if strcmp(ani_settings.graspable_points_in_reachable_area_show,'on')
    rot = rpy2dc([0;pi*inc/180;0])';
    
    % get an index of the swing leg
    i = gait_planning_param.swing_number;
    if ~isempty(gait_planning_param.graspable_points_in_reachable_area(:,:,i))
        graspable_points_in_reachable_area = rot'*gait_planning_param.graspable_points_in_reachable_area(:,:,i);
        
        for i = 1 : 1 : size(graspable_points_in_reachable_area,2)
            % if the contents is zero, it should not be drawn
            if sum(graspable_points_in_reachable_area(:,i)) ~= 0
                plot3(graspable_points_in_reachable_area(1,i), ...
                      graspable_points_in_reachable_area(2,i), ...
                      graspable_points_in_reachable_area(3,i), ...
                      ani_settings.graspable_points_in_reachable_area_marker,'Color',ani_settings.graspable_points_in_reachable_area_color,'MarkerSize',ani_settings.graspable_points_in_reachable_area_size)
            end
        end
end
end