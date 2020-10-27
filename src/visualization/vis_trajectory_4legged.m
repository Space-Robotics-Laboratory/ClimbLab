%%%%%% Results
%%%%%% vis_trajectory_4legged
%%%%%% 
%%%%%% Draw trajectory of 4 legged-robot
%%%%%% 
%%%%%% Created 2019-08-07
%%%%%% Warley Ribeiro
%%%%%% Last update: 2019-08-07
%
%
% Draw trajectory of all 4 legs of a robot
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT 
%         environment_param   : Parameters for environment related variables (struct)
%         ani_settings        : Animation settings (struct)
%         variables_saved     : Variables to be saved (struct)


function vis_trajectory_4legged(environment_param,ani_settings,variables_saved)

% Visualize Trajectory
if strcmp(ani_settings.trajectory_show,'on')
    color = ani_settings.trajectory_color;
    width = ani_settings.trajectory_width;
    line_type = ani_settings.trajectory_line_type;
    inc = environment_param.inc;
    
    trajectory1 = variables_saved.pos_e1';
    vis_trajectory(trajectory1,inc,color,width,line_type);
    trajectory2 = variables_saved.pos_e2';
    vis_trajectory(trajectory2,inc,color,width,line_type);
    trajectory3 = variables_saved.pos_e3';
    vis_trajectory(trajectory3,inc,color,width,line_type);
    trajectory4 = variables_saved.pos_e4';
    vis_trajectory(trajectory4,inc,color,width,line_type);
end


end