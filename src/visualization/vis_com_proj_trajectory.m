%%%%%% Results
%%%%%% vis_CoM_proj_trajectory
%%%%%% 
%%%%%% Draw trajectory of CoM projection
%%%%%% 
%%%%%% Created 2021-06-24
%%%%%% Keigo Haji
%%%%%% Last update: 2021-07-09
%%%%%% Keigo Haji
%
%
% Draw trajectory of CoM projection
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT 
%         gait_planning_param : Parameters for gait planning 
%         gait_planning_param.com_projection_history : 3*n matrix of com
%           projection history which is saved in vis_com_projection. The
%           thrid row is 0
%
%         ani_settings : Settings for animation
%
function vis_com_proj_trajectory(gait_planning_param, ani_settings, inc)

% Visualize Trajectory
if strcmp(ani_settings.com_proj_trajectory_show,'on')
    
    
    % Rotation matrix
    rot = rpy2dc([0;pi*inc/180;0])';
    % Rotate vectors
    traj = rot'* gait_planning_param.com_projection_history;
        
    color = ani_settings.com_proj_trajectory_color;
    width = ani_settings.com_proj_trajectory_width;
    line_type = ani_settings.com_proj_trajectory_line_type;
    
    plot3(traj(1,:),traj(2,:),traj(3,:), line_type,'Color',color,'LineWidth',width);
    
end

end

