%%%%%% Results
%%%%%% vis_vectors
%%%%%% 
%%%%%% Draw vectors based on the initial configuration
%%%%%% 
%%%%%% Created 2019-07-02
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-07-09
%%%%%% Keigo Haji
%
%
% Draw all selectable three-dimensional vectors
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT 
%         inc                    : Surface inclination [deg] (scalar)
%         ani_settings           : Animation settings (class)
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)
%		  LP                     : Link parameters (SpaceDyn class)
%         SV 	      			 : State variables (SpaceDyn class)
%         POS_e                  : Positions of end-effectors


function vis_vectors(inc,ani_settings,equilibrium_eval_param,LP,SV,POS_e,gait_planning_param,sensing_camera_param)

global Gravity

% Visualize Gravitational Acceralation
if strcmp(ani_settings.gravitational_acceleration_vec_show,'on')
    CoM = upd_CoM(LP, SV);
    color = ani_settings.gravitational_acceleration_vec_color;
    width = ani_settings.gravitational_acceleration_vec_width;
    magnitude = Gravity*ani_settings.acceleration_expansion_factor;
    arrow_head_size = 3.0;
    vis_one_vector(CoM,magnitude,inc,color,width,arrow_head_size);
end

% Visualize GIA
if strcmp(ani_settings.gia_vec_show,'on')
    CoM = upd_CoM(LP, SV);
    color = ani_settings.gia_vec_color;
    width = ani_settings.gia_vec_width;
    magnitude = equilibrium_eval_param.gia*ani_settings.acceleration_expansion_factor;
    arrow_head_size = 3.0;
    vis_one_vector(CoM,magnitude,inc,color,width,arrow_head_size);
end

% Visualize Gravitational Force
if strcmp(ani_settings.gravitational_force_vec_show,'on')
    CoM = upd_CoM(LP, SV);
    color = ani_settings.gravitational_force_vec_color;
    width = ani_settings.gravitational_force_vec_width;
    magnitude = Gravity*LP.mass*ani_settings.force_expansion_factor;
    arrow_head_size = 3.0;
    vis_one_vector(CoM,magnitude,inc,color,width,arrow_head_size);
end

% Visualize Reaction Force
if strcmp(ani_settings.reaction_force_show,'on')
    for i = 1:LP.num_limb
        color = ani_settings.reaction_force_vec_color;
        width = ani_settings.reaction_force_vec_width;
        arrow_head_size = 0.0;
        magnitude = SV.Fe(:,3*i)*ani_settings.force_expansion_factor;
        vis_one_vector(POS_e(:,i),magnitude,inc,color,width,arrow_head_size);
    end
end


end