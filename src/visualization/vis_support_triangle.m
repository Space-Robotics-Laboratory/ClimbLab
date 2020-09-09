%%%%%% Visualize
%%%%%% vis_support_triangle
%%%%%%
%%%%%% Disp support triangle
%%%%%%
%%%%%% Created 2020-05-14
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2020-05-14
%
%
% Disp gracpable points. Not considering gripping force.
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         SV                     : State Variables
%         POS_e                  : Position of end-effector
%         inc                    : Surface inclination [deg] (scalar)
%         ani_settings.support_triangle_show  'on', 'off'
%         ani_settings.support_triangle_color : Color of triangle [RGB] (1x3 vector)
%         ani_settings.support_triangle_alpha : Transparency of triangle (scalar)


function vis_support_triangle(SV, POS_e,inc, ani_settings)
if strcmp(ani_settings.support_triangle_show,'on')
    i=find(~SV.sup);
    p = POS_e;
    if ~isempty(i)
        p(:,i) = [];
    end
    p = rpy2dc([0;pi*inc/180;0])*p;
    fill3(p(1,:),p(2,:),p(3,:)+0.005, ani_settings.support_triangle_color, 'FaceAlpha',ani_settings.support_triangle_alpha);
end
end