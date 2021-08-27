%%%%%% Visualize
%%%%%% vis_support_triangle
%%%%%%
%%%%%% Disp support triangle
%%%%%%
%%%%%% Created 2020-05-14
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2021-05-17
%%%%%% Kentaro Uno
%
%
% Show the support polygon based on the contact positions, not considering gripping forces.
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         SV                     : State Variables
%         LP                     : Link Parameter
%         POS_e                  : Position of end-effector
%         inc                    : Surface inclination [deg] (scalar)
%         ani_settings.support_triangle_show  'on', 'off'
%         ani_settings.support_triangle_color : Color of triangle [RGB] (1x3 vector)
%         ani_settings.support_triangle_alpha : Transparency of triangle (scalar)
%         ani_settings.support_triangle_edge_color : edge line color of triangle [RGB] (1x3 vector)

function vis_support_triangle(SV, LP, POS_e, inc, ani_settings)

if strcmp(ani_settings.support_triangle_show,'on')
    i=find(~SV.sup);
    p = POS_e;
    p(3,:) = p(3,:) + 0.015; % support polygon should be drawn a bit higher not to be burried in the surface visualization
    if ~isempty(i)
        p(:,i) = [];
    end
    p = rpy2dc([0;pi*inc/180;0])*p;
    fill3(p(1,:),p(2,:),p(3,:), ani_settings.support_triangle_color, 'FaceAlpha', ani_settings.support_triangle_alpha, ... 
        'EdgeColor', ani_settings.support_triangle_edge_color);
end

if strcmp(ani_settings.robot_top_support_triangle_show,'on')
    i=find(~SV.sup);
    p = POS_e;
    CoM = upd_CoM(LP, SV);
%     p(3,:) = p(3,:) + 0.015; % support polygon should be drawn a bit higher not to be burried in the surface visualization
    if ~isempty(i)
        p(:,i) = [];
    end
    base_position_in_inertial_frame = rpy2dc([0;pi*inc/180;0]) * SV.R0;
    fill3(p(1,:)*cos(pi*inc/180),p(2,:),p(3,:)+base_position_in_inertial_frame(3)+2.0*CoM(3), ani_settings.support_triangle_color, 'FaceAlpha', ani_settings.support_triangle_alpha, ... 
        'EdgeColor', ani_settings.support_triangle_edge_color);
end


end