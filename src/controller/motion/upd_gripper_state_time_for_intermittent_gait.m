%%%%%% Update
%%%%%% upd_gripper_state_time_for_intermittent_gait
%%%%%% 
%%%%%% Change current gripper open/close state. For a gait that moves legs and base separately
%%%%%% 
%%%%%% Created 2020-04-17
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-12-21
%%%%%% Yusuke Koizumi
%
%
% Update the state of the gripper, if it is open or closed, based on the timing for the crawl gait
%
% Function variables:
%
%     OUTPUT
%         des_SV      			 : Desired state variables (SpaceDyn class)
%     INPUT
%         gait_planning_param    : Parameters for gait planning (class)
%         des_SV      			 : Desired state variables (SpaceDyn class)
%         time     				 : Simulation time [s] (scalar)

function des_SV = upd_gripper_state_time_for_intermittent_gait(gait_planning_param, des_SV, time)
if ~gait_planning_param.base_move
% Release gripper
des_SV.sup(gait_planning_param.swing_number) = 0;
% Close gripper
if rem(time,gait_planning_param.T_d) == 0 && time~=0
	des_SV.sup(gait_planning_param.swing_number) = 1;
end
end