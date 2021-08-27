%%%%%% Results
%%%%%% vis_stability_polyhedron
%%%%%% 
%%%%%% Draw stability polyhedron  (gia stable region) 
%%%%%% 
%%%%%% Created 2020-02-05
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-08-20
%%%%%% Warley Ribeiro
%
%
% Draw stablity polyhedron (gia stable region) for acceleration from intersection points with ground surface
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         inc                    : Surface inclination [deg] (scalar)
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)
%         ani_settings           : Animation settings (class)

function vis_stability_polyhedron(inc, equilibrium_eval_param, ani_settings)

if strcmp(ani_settings.gia_stable_region_show,'on') && equilibrium_eval_param.tumbling_axes_number > 1
    % Rotation matrix
    rot = rpy2dc([0;pi*inc/180;0])';
    % Rotate polyhedron to match surface inclination
    polyhedron.vertex = rot'* equilibrium_eval_param.polyhedron.vertex;

    for i = 1:size(polyhedron.vertex,2)-1
        patch('Vertices',[polyhedron.vertex(:,i) polyhedron.vertex(:,i+1) polyhedron.vertex(:,end)]',...
            'Faces',[1 2 3],'FaceAlpha',ani_settings.gia_stable_region_Face_alpha ,'Edgecolor',ani_settings.gia_stable_region_Edge_color ,'Facecolor',ani_settings.gia_stable_region_Face_color,'LineWidth',ani_settings.gia_stable_region_Edge_width)
    end
    patch('Vertices',[polyhedron.vertex(:,end-1) polyhedron.vertex(:,1) polyhedron.vertex(:,end)]',...
        'Faces',[1 2 3],'FaceAlpha',ani_settings.gia_stable_region_Face_alpha ,'Edgecolor',ani_settings.gia_stable_region_Edge_color ,'Facecolor',ani_settings.gia_stable_region_Face_color,'LineWidth',ani_settings.gia_stable_region_Edge_width)
end

end