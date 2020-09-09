%%%%%% Initialize
%%%%%% ini_joint_angle
%%%%%% 
%%%%%% Calculate the initial joints angles given the initial end-effectors' positions and map points
%%%%%% 
%%%%%% Created 2019-09-30
%%%%%% Victoria Keo, Warley Ribeiro
%%%%%% Last update: 2020-02-25
%
%
% For a given set of points xp, obtain the closest points in the map regardless of the map's resolution, including the map
% height for those points
%
% Function variables:
%
%     OUTPUT
%         q0         : Initial angular position for joints (1xn vector)
%     INPUT
%         LP         : Link Parameters (SpaceDyn class)
%         SV         : State Variables (SpaceDyn class)
%         foot_dist  : Horizontal distance from base center until end-effector position for both x and y coordinates [m] (scalar)


function q0 = ini_joint_angle(LP,SV,foot_dist)

% End-effectors initial (x,y) positions 
xe0 = [SV.R0(1)+foot_dist SV.R0(1)-foot_dist SV.R0(1)-foot_dist SV.R0(1)+foot_dist];
ye0 = [SV.R0(2)+foot_dist SV.R0(2)+foot_dist SV.R0(2)-foot_dist SV.R0(2)-foot_dist];
    
% Get the nearest points on the map and obtain height
[xe0,ye0,ze0] = get_map_pos(xe0,ye0) ;
    
POS_e0 = [xe0;ye0;ze0];
% Inverse kinematics
for i = 1:LP.num_limb
	q0(3*i-2:3*i) = get_i_kine_3dof(LP,SV,POS_e0(:,i) , i);
end
q0 = q0';

end