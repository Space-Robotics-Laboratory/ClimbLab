%%%%%% Results
%%%%%% vis_vectors
%%%%%% 
%%%%%% Draw vectors based on the initial configuration
%%%%%% 
%%%%%% Created 2019-07-02
%%%%%% Warley Ribeiro
%%%%%% Last update: 2019-07-02
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


function vis_vectors(inc,ani_settings,equilibrium_eval_param,LP,SV)

global Gravity

% Visualize Gravity
if strcmp(ani_settings.gravity_vec_show,'on')
    CoM = upd_CoM(LP, SV);
    color = ani_settings.gravity_vec_color;
    width = ani_settings.gravity_vec_width;
    magnitude = Gravity*equilibrium_eval_param.expansion_factor;
    vis_one_vector(CoM,magnitude,inc,color,width);
end

% Visualize GIA
if strcmp(ani_settings.gia_vec_show,'on')
    CoM = upd_CoM(LP, SV);
    color = ani_settings.gia_vec_color;
    width = ani_settings.gia_vec_width;
    magnitude = equilibrium_eval_param.gia*equilibrium_eval_param.expansion_factor;
    vis_one_vector(CoM,magnitude,inc,color,width);
end


end