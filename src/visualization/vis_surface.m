%%%%%% Visualize
%%%%%% vis_surface
%%%%%% 
%%%%%% Draw surface map
%%%%%% 
%%%%%% Created 2020-04-09
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-04-19
%%%%%% Keigo Haji
%
%
% Draw map from .mat file
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT 
%         inc                        : Surface inclination [deg] (scalar)
%         ani_settings.grid_color    : Color for surface grid
%         ani_settings.surface_color : Color for surface
%         environment_param.surface_type : surface name
%

function vis_surface(inc, ani_settings, environment_param, surface_param)
if strcmp(ani_settings.surface_show,'on')
    
% Global variables for map positions
global x; global y; global z
% Total number of points
n = size(z,1)*size(z,2); 
[X,Y] = meshgrid(x,y);
% Creat vectors array
MAP(1,:) = reshape(X,1,size(X,1)*size(X,2)); 
MAP(2,:) = reshape(Y,1,size(Y,1)*size(Y,2)); 
MAP(3,:) = reshape(z , 1 , n ); 

if strfind(environment_param.surface_type,'climbing_holds') > 0
    % Plot climbing holds map
        map = MAP;
        if inc ~= 0
        % Rotate map to match the surface inclination
        rot = rpy2dc([0;pi*inc/180;0])';
        map = rot' * MAP;
        end
        % Plot points by scatter3
        scatter3(map(1,:),map(2,:),map(3,:),'Marker','.','CData',MAP(3,:));
        colormap(ani_settings.surface_color);
        % Adjust colormap properly
        caxis([surface_param.min surface_param.max]);
else
    % Plot grid map
    if inc ~= 0
    % Rotate map to match the surface inclination
    rot = rpy2dc([0;pi*inc/180;0])'; 
    map = rot' * MAP; 
    % Change back from array to individual variables
    X = reshape(map(1,:),size(x,2),size(y,2)); 
    Y = reshape(map(2,:),size(x,2),size(y,2)); 
    Z = reshape(map(3,:),size(X,1),size(X,2)); 
    else
    Z = z;
    end
    % Plot map
    mesh(X,Y,Z,'edgecolor', ani_settings.grid_color);


    colormap(ani_settings.surface_color);
end
end
end