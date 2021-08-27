%%%%%% Visualize
%%%%%% vis_robot
%%%%%% 
%%%%%% Draw robot
%%%%%% 
%%%%%% Created: 2020-04-09
%%%%%% by Warley Ribeiro
%%%%%% Last update: 2020-01-19
%%%%%% by Kentaro Uno
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
%         ani_settings.robot_base_color : Color for robot base [RGB] (1x3 vector)
%         ani_settings.robot_base_alpha : Transparency for robot base [RGB] (1x3 vector)
%         ani_settings.robot_limb_color : Color for robot limb [RGB] (1x3 vector)
%         ani_settings.robot_limb_alpha : Transparency for robot limb [RGB] (1x3 vector)

function vis_robot(LP, SV, POS_e, shape_robot, inc, ani_settings)

% Rotation matrix
rot = rpy2dc([0;pi*inc/180;0])';

%% Draw base
% Change in the vertices of the base of the robot based on actual position and orientation
shape_robot.base_upper_V = shape_robot.base_upper_V*SV.A0' + ones(size(shape_robot.base_upper_V,1),1)*SV.R0';
shape_robot.base_upper_V = shape_robot.base_upper_V*rot;
shape_robot.base_lower_V = shape_robot.base_lower_V*SV.A0' + ones(size(shape_robot.base_lower_V,1),1)*SV.R0';
shape_robot.base_lower_V = shape_robot.base_lower_V*rot;

patch('Vertices',shape_robot.base_upper_V,'Faces',shape_robot.base_upper_F,'FaceColor',ani_settings.robot_base_upper_color,'Edgecolor','none',...
       'FaceAlpha',ani_settings.robot_base_alpha);
patch('Vertices',shape_robot.base_lower_V,'Faces',shape_robot.base_lower_F,'FaceColor',ani_settings.robot_base_lower_color,'Edgecolor','none',...
       'FaceAlpha',ani_settings.robot_base_alpha);
   
%% Draw the links    
% prepare the necessary vector for the f_kin_j
joints = [];
for j = 1:LP.num_q
    joints = [joints j];
end
[ POS_j, ORI_j ] = f_kin_j( LP, SV, joints );


% Draw the joint frames
joint_frame_x_axes = zeros( 3, length(joints) );
joint_frame_y_axes = zeros( 3, length(joints) );
joint_frame_z_axes = zeros( 3, length(joints) );
ex=[1;0;0]; ey=[0;1;0]; ez=[0;0;1];

if strcmp(ani_settings.frames_show,'on')

    % parameters for drawing the frames
    width = ani_settings.frames_line_width;
    magnitude = ani_settings.frames_size;
    arrow_head_size = 0.0; % not to visualize arrow head for simplicity
    % draw the frame at the CoM
    CoM = upd_CoM(LP, SV);
    vis_one_vector(CoM, magnitude*SV.A0*ex, inc, 'r', width, arrow_head_size);
    vis_one_vector(CoM, magnitude*SV.A0*ey, inc, 'g', width, arrow_head_size);
    vis_one_vector(CoM, magnitude*SV.A0*ez, inc, 'b', width, arrow_head_size);
    % draw the frames at each joint
    for joint_num = 1:LP.num_q
        joint_frame_x_axes(:,joint_num) = ORI_j(:,3*joint_num-2:3*joint_num)*ex;
        joint_frame_y_axes(:,joint_num) = ORI_j(:,3*joint_num-2:3*joint_num)*ey;
        joint_frame_z_axes(:,joint_num) = ORI_j(:,3*joint_num-2:3*joint_num)*ez;
        vis_one_vector(POS_j(:,joint_num), magnitude*joint_frame_x_axes(:,joint_num), inc, 'r', width, arrow_head_size);
        vis_one_vector(POS_j(:,joint_num), magnitude*joint_frame_y_axes(:,joint_num), inc, 'g', width, arrow_head_size);
        vis_one_vector(POS_j(:,joint_num), magnitude*joint_frame_z_axes(:,joint_num), inc, 'b', width, arrow_head_size);
    end
end 

% Draw the links
% position of the end-effector is adjusted for the inclination of the ground
POS_e = rot' * POS_e;

% necessary parameters for vis_cylinder()
r = ani_settings.link_radius; n = 7; closed=1; lines=0;
        
k = 1;
for i = 1:LP.num_q
        % position of each joint is adjusted for the inclination of the ground
        POS_j(:,i) = rot' * POS_j(:,i);
        % if the joint i is not connected the base (link index:0), connect 
        % the joint i and its parent joint, which number is obtained by LP.BB(i)
        if LP.BB(i) ~= 0
            [Cylinder EndPlate1 EndPlate2] = vis_cylinder(POS_j(:,i),POS_j(:,LP.BB(i)),...
                r,n,ani_settings.robot_limb_color,ani_settings.robot_limb_alpha,closed,lines);
        % moreover, if joint i is not the first joint, joint i-1 should be the
        % end-joint of the branch. -> connect it with the EE
        elseif i ~= 1
            [Cylinder EndPlate1 EndPlate2] = vis_cylinder(POS_j(:,i-1),POS_e(:,k),...
                r,n,ani_settings.robot_limb_color,ani_settings.robot_limb_alpha,closed,lines);
            k = k + 1;
        end
        % Finaslly, if joint i is the last joint, joint i should be also the
        % end-joint of the branch. -> connect it with the EE     
        if i == LP.num_q
            [Cylinder EndPlate1 EndPlate2] = vis_cylinder(POS_j(:,i),POS_e(:,k),...
                r,n,ani_settings.robot_limb_color,ani_settings.robot_limb_alpha,closed,lines);
        end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%   Old code for link visualization part
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %  Change in the vertices of each link of the robot based on actual position and orientation 
% k=1;
% for i = 1:LP.num_limb
%     for j = 1:3
%         shape_robot.link_V(:,:,k) = shape_robot.V_leg(:,:,j)*SV.AA(:,3*k-2:3*k)' + ...
%                                     ones(size(shape_robot.V_leg(:,:,1),1),1)*SV.RR(:,k)';
%         shape_robot.link_F(:,:,k) = shape_robot.F_leg(:,:,j);
%         k=k+1;
%     end
% end
% % Draw each link
% for i=1:LP.num_q
%     shape_robot.link_V(:,:,i) = shape_robot.link_V(:,:,i)*rot;
%     patch('Vertices',shape_robot.link_V(:,:,i),'Faces',shape_robot.link_F(:,:,i),'FaceColor',ani_settings.robot_color,...
%         'Edgecolor','none','FaceAlpha',ani_settings.robot_alpha);
% end

%% Draw grippers
if LP.F_grip > 0
%     POS_e = rot'* POS_e;
    for i = 1:LP.num_limb
        shape_robot.gripper_V(:,:,i) = shape_robot.grip_V*rot + ones(size(shape_robot.grip_V,1),1)*POS_e(1:3,i)';
        patch('Vertices',shape_robot.gripper_V(:,:,i),'Faces',shape_robot.grip_F,'FaceColor',ani_settings.robot_limb_color,...
            'Edgecolor','none','FaceAlpha',ani_settings.robot_limb_alpha);
        shape_robot.gripper_V(:,:,i) = shape_robot.grip_V*rpy2dc([0;0;pi/2])*rot + ones(size(shape_robot.grip_V,1),1)*POS_e(1:3,i)';
        patch('Vertices',shape_robot.gripper_V(:,:,i),'Faces',shape_robot.grip_F,'FaceColor',ani_settings.robot_limb_color,...
            'Edgecolor','none','FaceAlpha',ani_settings.robot_limb_alpha);
    end
end