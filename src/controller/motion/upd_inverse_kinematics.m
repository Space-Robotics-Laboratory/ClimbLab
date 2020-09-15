%%%%%% Update
%%%%%% upd_inverse_kinematics
%%%%%% 
%%%%%% Obtain inverse kinematics
%%%%%% 
%%%%%% Created 2020-04-17
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-17
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
    des_SV.q(3*i-2:3*i) = get_i_kine_3dof(LP, des_SV, motion_planning_param.des_POS_e(:,i), i);
end
