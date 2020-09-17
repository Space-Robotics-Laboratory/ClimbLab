%%%%%% Results
%%%%%% vis_one_vector
%%%%%% 
%%%%%% Draw a vector
%%%%%% 
%%%%%% Created 2019-10-31
%%%%%% Warley Ribeiro
%%%%%% Last update: 2019-07-02
%
%
% Draw a three-dimensional vector from a specific origin point
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT 
%         origin       : Origin point for the vector (3x1 vector)
%         vector       : Vector to be plotted (3x1 vector)
%         inc          : Inclination [deg] (scalar)
%         color        : Color for vector [RGB] (1x3 vector)
%         width        : Width for vector line (scalar)


function vis_one_vector(origin,vector,inc,color,width)

% Rotation matrix
rot = rpy2dc([0;pi*inc/180;0])';
% Rotate vectors
origin = rot'* origin;
vector = rot'* vector;
% Initial and final points of the vector
VEC = [origin vector+origin];

plot3(VEC(1,:),VEC(2,:),VEC(3,:),'-','Color',color,'LineWidth',width);
hold on
% Arrow head
plot3(VEC(1,2),VEC(2,2),VEC(3,2),'v','Color',color,'MarkerFaceColor',color,'LineWidth',width,'MarkerSize',3);

end