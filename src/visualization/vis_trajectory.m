%%%%%% Results
%%%%%% vis_trajectory
%%%%%% 
%%%%%% Draw a trajectory
%%%%%% 
%%%%%% Created 2019-08-07
%%%%%% Warley Ribeiro
%%%%%% Last update: 2019-09-08
%
%
% Draw a trajectory
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT 
%         trajectory   : Points of the trajectory (3xn matrix)
%         inc          : Inclination [deg] (scalar)
%         color        : Color for vector [RGB] (1x3 vector)
%         width        : Line width (scalar)
%         line_type    : Line type (string)


function vis_trajectory(trajectory,inc,color,width,line_type)

% Rotation matrix
rot = rpy2dc([0;pi*inc/180;0])';
% Rotate vectors
traj = rot'* trajectory;

plot3(traj(1,:),traj(2,:),traj(3,:),line_type,'Color',color,'LineWidth',width);

end