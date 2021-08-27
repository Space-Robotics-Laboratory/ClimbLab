%%%%%% Update
%%%%%% upd_gripper_state_time
%%%%%% 
%%%%%% Change current gripper open/close state
%%%%%% 
%%%%%% Created 2020-04-17
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-04-26
%%%%%% Warley Ribeiro
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

function des_SV = upd_gripper_state_time(gait_planning_param, LP, des_SV, time)

global d_time

for i = 1:LP.num_limb
    if i == gait_planning_param.swing_number
        % Release gripper of swing limb
        des_SV.sup(i) = 0;
        % Close gripper of swing limb when reaching next grasping position
        if size(gait_planning_param.leg_T,2) > 1
            close_timing = gait_planning_param.leg_T(2,i);
        else
            close_timing = gait_planning_param.leg_T(2);
        end
        if time >= (close_timing - d_time) && time~=0
            des_SV.sup(i) = 1;
        end
    end
end

