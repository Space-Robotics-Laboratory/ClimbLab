%%%%%% Initialize
%%%%%% ini_reachable_area
%%%%%% 
%%%%%% Calculate the reachable area and max./min. range from CoM
%%%%%% 
%%%%%% Created 2020-05-13
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2020-06-24 by Kentaro Uno (changed the joint limitaion)
%
%
% Calculate a reachable area and and max./min. range from CoM
%
% Function variables:
%
%     OUTPUT
%         LP.reachable_area.min/max         : Minimum/Maximum range from CoM
%         LP.reachable_area.data_near/far   : Data of the position vectors that can be reached by swing leg(if necessary)
%     INPUT
%         LP          : Link Parameters (SpaceDyn class)
%         SV          : State Variables (SpaceDyn class)
%         floor       : height of floor  [m] (scalar)
%         joint_limit : min./max. position of joints in deg. (3x2 matrix)
%                       HubRobo : joint1;joint2;joint3 => [-60 60; 0 90; -135 0]




function [ LP ] = ini_reachable_area(LP,SV,floor,joint_limit)
% set the reachable area from the limitations of the motion of joint servo motor

LP.joint_limit=joint_limit;
SV.R0=zeros(3,1);
% set the initial joint angles of the robot
SV.q = zeros(12,1);

cnt = 1;
for j=  joint_limit(2,1):0.5:joint_limit(2,2)    % change of the link 2 when link 3 bends up maximumly
SV.q = zeros(12,1);
SV.q(1) = 45*pi/180;
SV.q(2) = j*pi/180;
SV.q(3) = joint_limit(3,2)*pi/180;

SV = calc_aa( LP, SV );
SV = calc_pos( LP, SV );

% calculation of the position of the gripper
[ POS_e1, ORI_e1 ] = f_kin_e(LP, SV, 3 );

RA.POS_e1_tmp1(cnt,:) = POS_e1';
cnt = cnt + 1;
end


cnt = 1;
for j=  joint_limit(2,1):0.5:joint_limit(2,2)    % change of the link 2 when link 3 bends down minimumly
SV.q = zeros(12,1);
SV.q(1) = 45*pi/180;
SV.q(2) = j*pi/180;
SV.q(3) = joint_limit(3,1)*pi/180;

SV = calc_aa( LP, SV );
SV = calc_pos( LP, SV );

% calculation of the position of the gripper
[ POS_e1, ORI_e1 ] = f_kin_e(LP, SV, 3 );

RA.POS_e1_tmp2(cnt,:) = POS_e1';
cnt = cnt + 1;
end

cnt = 1;
for j=  joint_limit(3,1):0.5:joint_limit(3,2)    % change of the link 3 when link 2 bends down minimumly

SV.q = zeros(12,1);
SV.q(1) = 45*pi/180;
SV.q(2) = joint_limit(2,1)*pi/180;
SV.q(3) = j*pi/180;

SV = calc_aa( LP, SV );
SV = calc_pos( LP, SV );

% calculation of the position of the gripper
[ POS_e1, ORI_e1 ] = f_kin_e(LP, SV, 3 );

RA.POS_e1_tmp3(cnt,:) = POS_e1';
cnt = cnt + 1;
end

cnt = 1;
for j=  joint_limit(3,1):0.5:joint_limit(3,2)    % change of the link 3 when link 2 bends up maximumly

SV.q = zeros(12,1);
SV.q(1) = 45*pi/180;
SV.q(2) = joint_limit(2,2)*pi/180;
SV.q(3) = j*pi/180;

SV = calc_aa( LP, SV );
SV = calc_pos( LP, SV );

% calculation of the position of the gripper
[ POS_e1, ORI_e1 ] = f_kin_e(LP, SV, 3 );

RA.POS_e1_tmp4(cnt,:) = POS_e1';
cnt = cnt + 1;
end
data_near =[RA.POS_e1_tmp2' RA.POS_e1_tmp4'];
data_far  =[RA.POS_e1_tmp3' RA.POS_e1_tmp1'];
[~,I_min] = min(data_far(3,:));

LP.reachable_area.data_near =[fliplr(data_far(:,1:I_min)) data_near ];
LP.reachable_area.data_far  =[data_far(:,I_min:length(data_far))];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% find the points where z = -0.08 (ground) out of dataset %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
diff1 = abs( LP.reachable_area.data_far(3,:) - floor );
diff2 = abs( LP.reachable_area.data_near(3,:) - floor );
[~,I_1] = min(diff1(:));
[~,I_2] = min(diff2(:));

    x1 = LP.reachable_area.data_far(2,I_1(1));
%     y1 = RA.POS_e1_tmp3(I_1(1),3);
    x2 = LP.reachable_area.data_near(2,I_2(1));
%     y2 = RA.POS_e1_tmp2(I_2(1),3);

LP.reachable_area.max = x1;
LP.reachable_area.min = x2;

end