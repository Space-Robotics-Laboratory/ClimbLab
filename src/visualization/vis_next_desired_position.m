%%%%%% Visualize
%%%%%% vis_next_desired_position
%%%%%%
%%%%%% Disp next desired position
%%%%%%
%%%%%% Created 2020-05-28
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2020-05-28
%
%
% Display the next selected position to be grasped
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         path_planning_param.POS_next             : Next desired position of end-effector
%         inc                                      : Surface inclination [deg] (scalar)
%         ani_settings.next_desired_position_color : Color for points [RGB] (1x3 vector)
%         ani_settings.next_desired_position_marker: Type of marker (String)
%         ani_settings.next_desired_position_show on, off



function vis_next_desired_position(path_planning_param, inc, ani_settings)
if strcmp(ani_settings.next_desired_position_show,'on')
    dr = 0.015;
    dz = 0;
    des_pos_now = path_planning_param.POS_next(:,path_planning_param.swing_number);
    des_pos_square(:,1) = des_pos_now + [dr; dr; dz];
    des_pos_square(:,2) = des_pos_now + [dr;-dr; dz];
    des_pos_square(:,3) = des_pos_now + [-dr;-dr;dz];
    des_pos_square(:,4) = des_pos_now + [-dr;dr; dz];
    des_pos_square(:,5) = des_pos_square(:,1);
    des_pos_square = rpy2dc([0;pi*inc/180;0])*des_pos_square;
    plot3(des_pos_square(1,:), des_pos_square(2,:), des_pos_square(3,:),ani_settings.next_desired_position_color,...
        'LineWidth',ani_settings.next_desired_position_line_width);
end
end