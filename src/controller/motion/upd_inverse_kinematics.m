%%%%%% Update
%%%%%% upd_inverse_kinematics
%%%%%% 
%%%%%% Obtain inverse kinematics
%%%%%% 
%%%%%% Created 2020-04-17
%%%%%% by Warley Ribeiro
%%%%%% Last update: 2021-02-12
%%%%%% by Kentaro Uno
%
%
% Update desired joint space configuration state from inverse kinematics
%
% Function variables:
%
%     OUTPUT
%         des_SV                : Desired state variables (class)
%     INPUT
%         motion_planning_param : Parameters for motion planning (class)
%		  LP                    : Link parameters (SpaceDyn class)
%		  des_SV                : Desired state variables (SpaceDyn class)

function des_SV = upd_inverse_kinematics(motion_planning_param, LP, des_SV)

% Update desired base POS
des_SV.R0 = motion_planning_param.des_R0';
for i = 1:LP.num_limb
    
    if LP.joint_allocation_type == 'mammal'
        des_SV.q(3*i-2:3*i) = get_i_kine_mammal_config_3dof_limb(LP, des_SV, motion_planning_param.des_POS_e(:,i), i);
    elseif LP.joint_allocation_type == 'insect'
        des_SV.q(3*i-2:3*i) = get_i_kine_insect_config_3dof_limb(LP, des_SV, motion_planning_param.des_POS_e(:,i), i);
    end
    
%     if  LP.joint_limit(1,1)>= rad2deg( des_SV.q(3*i-2) ) || rad2deg( des_SV.q(3*i-2) ) >= LP.joint_limit(1,2) || ...
%         LP.joint_limit(2,1)>= rad2deg( des_SV.q(3*i-1) ) || rad2deg( des_SV.q(3*i-1) ) >= LP.joint_limit(2,2) || ...
%         LP.joint_limit(3,1)>= rad2deg( des_SV.q(3*i  ) ) || rad2deg( des_SV.q(3*i  ) ) >= LP.joint_limit(3,2)
%         disp("the number of limb that had a IK problem: ");
%         disp( i );
%         disp("the solution of IK is as follows [deg]: ");
%         disp( rad2deg(des_SV.q(3*i-2:3*i)) );
%         error("in upd_inverse_kinematics(): the solution of IK escapes from the LP.joint_limitation");
%     end 
end
