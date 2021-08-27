%%%%%% Results
%%%%%% vis_trajectory_4legged
%%%%%% 
%%%%%% Draw trajectory of 4 legged-robot
%%%%%% 
%%%%%% Created 2019-08-07
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-04-26
%%%%%% Kentaro Uno
%
%
% Draw trajectory of all 4 legs of a robot
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT 
%         environment_param      : Parameters for environment related variables (struct)
%         motion_planning_param  : Motiion planning parameters (struct)
%           motion_planning_param.trajectories.[LF, LH, RH, RF]:
%               trajectories of EE updated in upd_motion_planning
%         ani_settings           : Animation settings (struct)


function vis_trajectory_4legged(environment_param, motion_planning_param, ani_settings)

% Visualize Trajectory
if strcmp(ani_settings.trajectory_show,'on')

    color = ani_settings.trajectory_color;
    width = ani_settings.trajectory_width;
    line_type = ani_settings.trajectory_line_type;
    inc = environment_param.inc;
    
    vis_trajectory(motion_planning_param.trajectories.LF, inc, color, width, line_type);
    vis_trajectory(motion_planning_param.trajectories.LH, inc, color, width, line_type);
    vis_trajectory(motion_planning_param.trajectories.RH, inc, color, width, line_type);
    vis_trajectory(motion_planning_param.trajectories.RF, inc, color, width, line_type);
end

end