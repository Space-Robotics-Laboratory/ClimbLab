%%%%%% Update
%%%%%% upd_base_ori
%%%%%%
%%%%%% Calculate current desired orientation for base 
%%%%%%
%%%%%% Created 2020-04-14
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2020-12-21
%
%
% Calculate orientation for base 
%
% Function variables:
%
%     OUTPUT
%         des_SV                 : Desired state variables (SpaceDyn class)
%     INPUT
%         des_SV                 : Desired state variables (SpaceDyn class)
%         gait_planning_param    : Parameters for motion planning (class)
%         LP      				 : Link Parameters (SpaceDyn class)
%         time     				 : Simulation time [s] (scalar)

function des_SV = upd_base_ori(des_SV, gait_planning_param, LP, time)
if gait_planning_param.base_move
	des_SV.Q0 =  ((gait_planning_param.base_orientation_next-gait_planning_param.base_orientation)*(time - gait_planning_param.base_T(1))/(gait_planning_param.base_T(2) - gait_planning_param.base_T(1))) + gait_planning_param.base_orientation;
    des_SV.A0 = rpy2dc(des_SV.Q0)';
end

end