%%%%%% Visualize
%%%%%% vis_com_projection
%%%%%%
%%%%%% Display CoM projection point
%%%%%%
%%%%%% Created 2020-05-28
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2020-05-28
%
%
% Display Center of Mass projection
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         SV                                 : State Variables
%         inc                                : Surface inclination [deg] (scalar)
%         ani_settings.com_projection_color  : Color for points [RGB] (1x3 vector)
%         ani_settings.com_projection_marker : Type of marker (String)
%         ani_settings.com_projection_size   : Size of points (scalar)
%         ani_settings.com_projection_show   : on / off


function vis_com_projection(SV, inc, ani_settings)
if strcmp(ani_settings.com_projection_show,'on')
    [a,b,c] =  get_map_pos(SV.R0(1),SV.R0(2));
    com_proj = rpy2dc([0;pi*inc/180;0])*[SV.R0(1,1)-norm(c)*tan(inc*pi/180);SV.R0(2,1);c];
    plot3(com_proj(1,1),com_proj(2,1),com_proj(3,1),ani_settings.com_projection_marker,'Color',ani_settings.com_projection_color,'MarkerSize',ani_settings.com_projection_size);
end
end