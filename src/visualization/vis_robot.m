%%%%%% Visualize
%%%%%% vis_robot
%%%%%% 
%%%%%% Draw robot
%%%%%% 
%%%%%% Created 2020-04-09
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-17
%
%
% Draw robot from current parameters and from links' simplified shapes
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT 
%         LP          : Link Parameters (SpaceDyn class)
%         SV          : State Variables (SpaceDyn class)
%         F_grip      : Maximum gripping force [N] (scalar)
%         POS_e       : End-effector positions (3xnum_limb matrix) 
%         shape_robot : Vertices and faces indicators for base and legs (structure)
%         inc         : Surface inclination [deg] (scalar)
%         ani_settings.robot_color : Color for robot [RGB] (1x3 vector)


function vis_robot(LP, SV, F_grip, POS_e, shape_robot, inc, ani_settings)

% Rotation matrix
rot = rpy2dc([0;pi*inc/180;0])';

% Change in the vertices of the base of the robot based on actual position and orientation
shape_robot.base_V = shape_robot.base_V*SV.A0' + ones(size(shape_robot.base_V,1),1)*SV.R0';
shape_robot.base_V = shape_robot.base_V*rot;
% Draw base
patch('Vertices',shape_robot.base_V,'Faces',shape_robot.base_F,'FaceColor',ani_settings.robot_color,'Edgecolor','none',...
       'FaceAlpha',ani_settings.robot_alpha);

%  Change in the vertices of each link of the robot based on actual position and orientation 
k=1;
for i = 1:LP.num_limb
    for j = 1:3
        shape_robot.link_V(:,:,k) = shape_robot.V_leg(:,:,j)*SV.AA(:,3*k-2:3*k)' + ...
                                    ones(size(shape_robot.V_leg(:,:,1),1),1)*SV.RR(:,k)';
        shape_robot.link_F(:,:,k) = shape_robot.F_leg(:,:,j);
        k=k+1;
    end
end
% Draw each link
for i=1:LP.num_q
    shape_robot.link_V(:,:,i) = shape_robot.link_V(:,:,i)*rot;
    patch('Vertices',shape_robot.link_V(:,:,i),'Faces',shape_robot.link_F(:,:,i),'FaceColor',ani_settings.robot_color,...
        'Edgecolor','none','FaceAlpha',ani_settings.robot_alpha);
end

% Draw grippers
if F_grip > 0
    POS_e = rot'* POS_e;
    for i = 1:LP.num_limb
        shape_robot.gripper_V(:,:,i) = shape_robot.grip_V*rot + ones(size(shape_robot.grip_V,1),1)*POS_e(1:3,i)';
        patch('Vertices',shape_robot.gripper_V(:,:,i),'Faces',shape_robot.grip_F,'FaceColor',ani_settings.robot_color,...
            'Edgecolor','none','FaceAlpha',ani_settings.robot_alpha);
        shape_robot.gripper_V(:,:,i) = shape_robot.grip_V*rpy2dc([0;0;pi/2])*rot + ones(size(shape_robot.grip_V,1),1)*POS_e(1:3,i)';
        patch('Vertices',shape_robot.gripper_V(:,:,i),'Faces',shape_robot.grip_F,'FaceColor',ani_settings.robot_color,...
            'Edgecolor','none','FaceAlpha',ani_settings.robot_alpha);
    end
end