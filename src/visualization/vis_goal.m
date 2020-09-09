%%%%%% Visualize
%%%%%% vis_goal
%%%%%%
%%%%%% Disp goal
%%%%%%
%%%%%% Created 2020-05-28
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2020-05-28
%
%
% Disp goal
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         gait_param.goal         : Position vector of goal
%         inc                     : Surface inclination [deg] (scalar)
%         ani_settings.goal_color : Color for points [RGB] (1x3 vector)
%         ani_settings.goal_marker: Type of marker (String)
%         ani_settings.goal_size  : Size of points (scalar)
%         ani_settings.goal_show  : 'on','off'

function vis_goal(gait_param, inc, ani_settings)
if strcmp(ani_settings.goal_show,'on')
    goal = rpy2dc([0;pi*inc/180;0]) * gait_param.goal;
    plot3(goal(1,:),goal(2,:),goal(3,:),ani_settings.goal_marker,'Color',ani_settings.goal_color,'MarkerSize',ani_settings.goal_size);   
end
end