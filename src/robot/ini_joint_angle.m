%%%%%% Initialize
%%%%%% ini_joint_angle
%%%%%% 
%%%%%% Calculate the initial joints angles given the initial end-effectors' positions and map points
%%%%%% 
%%%%%% Created 2019-09-30
%%%%%% by Victoria Keo, Warley Ribeiro
%%%%%% Last update: 2021-05-17
%%%%%% by Kentaro Uno
%
%
% For a given set of points xp, obtain the closest points in the map regardless of the map's resolution, including the map
% height for those points
%
% Function variables:
%
%     OUTPUT
%         q0          : Initial angular position for joints (1xn vector)
%     INPUT
%         LP          : Link Parameters (SpaceDyn class)
%         SV          : State Variables (SpaceDyn class)
%         robot_param  : Parameter for robot (class)
%         x_foot_dist : Horizontal distance from base center until end-effector position for both x and y coordinates [m] (scalar)
%         y_foot_dist : same as above
%         surface_param         : Parameters for surface (class)
%           surface_param.graspable_points : 3xN matrix contains position vectors of graspable points [m] (x;y;z)
    

function q0 = ini_joint_angle(LP,SV,robot_param,x_foot_dist,y_foot_dist,surface_param)

% rotate the initial foot position based on the robot_param.initial_yaw
re0 = rpy2dc([0;0;-pi*robot_param.initial_yaw/180]) * [+x_foot_dist -x_foot_dist -x_foot_dist +x_foot_dist; ...
                                                       +y_foot_dist +y_foot_dist -y_foot_dist -y_foot_dist; ...
                                                       0 0 0 0];
% End-effectors initial (x,y) positions 
xe0 = [SV.R0(1)+re0(1,1) SV.R0(1)+re0(1,2) SV.R0(1)+re0(1,3) SV.R0(1)+re0(1,4)];
ye0 = [SV.R0(2)+re0(2,1) SV.R0(2)+re0(2,2) SV.R0(2)+re0(2,3) SV.R0(2)+re0(2,4)];

xe0 = xe0 - robot_param.x_base_pos_offset_from_the_neutral_pos;
ye0 = ye0 - robot_param.y_base_pos_offset_from_the_neutral_pos;

% Get the nearest points on the map and obtain height
[xe0,ye0,ze0] = get_map_pos_of_graspable_points(xe0,ye0,surface_param) ;
    
POS_e0 = [xe0;ye0;ze0];
% Inverse kinematics
for i = 1:LP.num_limb
    if LP.joint_allocation_type == 'mammal'
        q0(3*i-2:3*i) = get_i_kine_mammal_config_3dof_limb(LP,SV,POS_e0(:,i) , i);
    elseif LP.joint_allocation_type == 'insect'
        q0(3*i-2:3*i) = get_i_kine_insect_config_3dof_limb(LP,SV,POS_e0(:,i) , i);
    end
    if  LP.joint_limit(1,1)>= rad2deg( q0(3*i-2) ) || rad2deg( q0(3*i-2) ) >= LP.joint_limit(1,2) || ...
        LP.joint_limit(2,1)>= rad2deg( q0(3*i-1) ) || rad2deg( q0(3*i-1) ) >= LP.joint_limit(2,2) || ...
        LP.joint_limit(3,1)>= rad2deg( q0(3*i  ) ) || rad2deg( q0(3*i  ) ) >= LP.joint_limit(3,2)
        disp("the solution of IK is as follows [deg]");
        disp( rad2deg(q0(3*i-2:3*i)) );
        error("in ini_joint_angle(): the solution of IK escapes from the LP.joint_limitation");
    end 
end
q0 = q0';

end