%%%%%% Update
%%%%%% upd_gripper_state_time
%%%%%% 
%%%%%% Change current gripper open/close state
%%%%%% 
%%%%%% Created 2020-04-17
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-17
%
%
% Update the state of the gripper, if it is open or closed, based on the timing for the crawl gait
%
% Function variables:
%
%     OUTPUT
%         des_SV      			 : Desired state variables (SpaceDyn class)
%     INPUT
%         gait_param             : Parameters for gait (class)
%         path_planning_param    : Parameters for path planning (class)
%         des_SV      			 : Desired state variables (SpaceDyn class)
%         time     				 : Simulation time [s] (scalar)

function des_SV = upd_gripper_state_time(gait_param, path_planning_param, des_SV, time)

% Release gripper
des_SV.sup(path_planning_param.swing_number) = 0;
% Close gripper
if rem(time,gait_param.T_d) == 0 && time~=0
	des_SV.sup(path_planning_param.swing_number) = 1;
end
