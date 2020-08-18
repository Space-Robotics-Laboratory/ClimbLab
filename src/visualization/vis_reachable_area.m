%%%%%% Visualize
%%%%%% vis_reachable_area
%%%%%%
%%%%%% Disp reachable area
%%%%%%
%%%%%% Created 2020-05-20
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2020-07-11 by Kentaro Uno
%
%
% Disp reachable points
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         SV                                     : State Variables
%         LP                                     : Link Parameters
%         inc                                    : Surface inclination [deg] (scalar)
%         ani_settings.reachable_area_line_color : Color for line [RGB] (1x3 vector)
%         ani_settings.reachable_area_line_width : Line width of marker (scalar)
%         ani_settings.reachable_area_show       : 'on','off' (String)
%         surface_param.floor_level              : floor z-directional
%                                                  position seen from the base



function vis_reachable_area(SV, LP, path_planning_param, inc, ani_settings, surface_param)
if strcmp(ani_settings.reachable_area_show, 'on')
    % get an index of the swing leg
    i = path_planning_param.swing_number;
    
    alpha = LP.Qi(3, (3*i-2));
    joint = SV.R0 + SV.A0*LP.c0(:,3*i-2);
    r_min = LP.reachable_area.min-LP.c0(1,1);
    r_max = LP.reachable_area.max-LP.c0(1,1);
    
    t = linspace(LP.joint_limit(1,1)*pi/180,LP.joint_limit(1,2)*pi/180,10);
    % Arc
    X1 = [r_min*cos(t);r_min*sin(t);ones(1,length(t))*surface_param.floor_level];
    X2 = [r_max*cos(-t);r_max*sin(-t);ones(1,length(t))*surface_param.floor_level];
    X1 = rot_z(alpha)*X1+[joint(1:2);0];
    X2 = rot_z(alpha)*X2+[joint(1:2);0];
    
    X1 = rpy2dc([0;pi*inc/180;0])*X1;
    X2 = rpy2dc([0;pi*inc/180;0])*X2;
    
    plot3([X1(1,:) X2(1,:) X1(1,1)],[X1(2,:) X2(2,:) X1(2,1)],[X1(3,:) X2(3,:) X1(3,1)]+0.005,'Color',ani_settings.reachable_area_line_color,'LineWidth',ani_settings.reachable_area_line_width);
end
end