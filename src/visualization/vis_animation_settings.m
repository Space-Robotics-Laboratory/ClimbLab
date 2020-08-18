%%%%%% Visualize
%%%%%% vis_animation_settings
%%%%%% Set figure parameters for 3D graphs
%%%%%% 
%%%%%% Created 2020-04-09
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-09
%
%
% Modify three-dimensional figure parameters
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT 
%         ani_settings.font_name             : Name of the font (string)
%         ani_settings.font_size             : Size of the font (scalar)
%         ani_settings.animation_resolution  : Pixel resolution (1x2 vector)
%         ani_settings.x_lim                 : Limits for the x-axis (1x2 vector)
%         ani_settings.y_lim                 : Limits for the y-axis (1x2 vector)
%         ani_settings.z_lim                 : Limits for the z-axis (1x2 vector)
%         ani_settings.z_lim_low_is_surface  : Use or not surface as lower limit for z-axis (string)
%         ani_settings.camera_az             : Azimuth angle for view [deg] (scalar)
%         ani_settings.camera_el             : Elevation angle for view [deg] (scalar)


function vis_animation_settings(ani_settings, surface_param, time)

%%% Definition of files for writing simulation results %%%
% Labels
xlabel({'\it{x} \rm{[m]}';['Time: ',num2str(time,'%.2f'), ' [s]']},'FontName',ani_settings.font_name,'FontSize',ani_settings.font_size);
ylabel('\it{y} \rm{[m]}','FontName',ani_settings.font_name,'FontSize',ani_settings.font_size);
zlabel('\it{z} \rm{[m]}','FontName',ani_settings.font_name,'FontSize',ani_settings.font_size);
% Define fonts
set(gca,'FontName',ani_settings.font_name,'FontSize',ani_settings.font_size,'LineWidth',2);
% Define background color and window size (resolution)
set(gcf,'color','w','Position', [1 1 ani_settings.animation_resolution]);
set(gcf,'Renderer','zbuffer')
% Axis, grid and camera angle
axis equal; grid on; view(ani_settings.camera_az,ani_settings.camera_el);
% Limits for axes
if strcmp(ani_settings.z_lim_low_is_surface,'on') && strcmp(ani_settings.move_camera,'off')
	ani_settings.z_lim(1) = surface_param.min;
end
xlim(ani_settings.x_lim); ylim(ani_settings.y_lim); zlim(ani_settings.z_lim);
% Lighting paramenters
lighting gouraud; material shiny; lightangle(-10,15)

end