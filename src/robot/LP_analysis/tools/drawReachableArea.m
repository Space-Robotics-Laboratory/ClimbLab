%%%%%% Reachbility simulation
%%%%%%
%%%%%% function name:
%%%%%% reachable_area.m
%%%%%% 
%%%%%% objective:
%%%%%% to obtain cross-sectional 2D reachable area of the endpoint (gripper)
%%%%%% of the insect-typed 3 DOF leg 
%%%%%% 
%%%%%% input: 
%%%%%% - LP: link parameters class of the robot 
%%%%%% - SV: states valuables 
%%%%%% - joint1: fixed value of joint1 to make it to aim towards + x direction 
%%%%%% - joint2 and 3: min and max value of the joint 2 and 3
%%%%%% - color: visualization of the color
%%%%%% - fig_num: figure number 
%%%%%%
%%%%%% output:
%%%%%% - reachable area vizualization
%%%%%%
%%%%%% Created Date:
%%%%%% 2019-09-10
%%%%%% Updated Date:
%%%%%% 2019-06-21
%%%%%% Auther: 
%%%%%% Kentaro Uno ( based on Hayato Minote's code )

function [  ] = reachable_area( LP , SV , joint1 , joint2 , joint3, color, fig_num, robot_name)
% belief: set the reachable area from the limitations of the motion of joint servo motor
 
% set the initial joint angles of the robot 
SV.q = zeros(12,1);

%% 1st loop
cnt = 1;
for j=  joint2.min : 0.5 : joint2.max    % change of the link 2 when link 3 bends up maximumly
SV.q = zeros(12,1);
SV.q(1) = joint1;
SV.q(2) = j*pi/180;
SV.q(3) = joint3.max*pi/180;

SV = calc_aa( LP, SV );
SV = calc_pos( LP, SV );

% calculation of the position of the gripper
[ POS_e1, ORI_e1 ] = f_kin_e(LP, SV, 3 );

POS_e1_tmp1(cnt,:) = POS_e1';
cnt = cnt + 1;
end

%% 2nd loop
cnt = 1;
for j=  joint2.min : 0.5 : joint2.max    % change of the link 2 when link 3 bends down maximumly
SV.q = zeros(12,1);
SV.q(1) = joint1;
SV.q(2) = j*pi/180;
SV.q(3) = joint3.min*pi/180;

SV = calc_aa( LP, SV );
SV = calc_pos( LP, SV );

% calculation of the position of the gripper
[ POS_e1, ORI_e1 ] = f_kin_e(LP, SV, 3 );

POS_e1_tmp2(cnt,:) = POS_e1';
cnt = cnt + 1;
end

%% 3rd loop
cnt = 1;
for j=  joint3.min : 0.5 : joint3.max    % change of the link 3 when link 2 has 0 degree

SV.q = zeros(12,1);
SV.q(1) = joint1;
SV.q(2) = joint2.min*pi/180;
SV.q(3) = j*pi/180;

SV = calc_aa( LP, SV );
SV = calc_pos( LP, SV );

% calculation of the position of the gripper
[ POS_e1, ORI_e1 ] = f_kin_e(LP, SV, 3 );

POS_e1_tmp3(cnt,:) = POS_e1';
cnt = cnt + 1;
end

%% 4th loop
cnt = 1;
for j=  joint3.min : 0.5 : joint3.max    % change of the link 3 when link 2 bends up maximumly

SV.q = zeros(12,1);
SV.q(1) = joint1;
SV.q(2) = joint2.max*pi/180;
SV.q(3) = j*pi/180;

SV = calc_aa( LP, SV );
SV = calc_pos( LP, SV );

% calculation of the position of the gripper
[ POS_e1, ORI_e1 ] = f_kin_e(LP, SV, 3 );

POS_e1_tmp4(cnt,:) = POS_e1';
cnt = cnt + 1;
end

% reset all state valuables
SV.q = zeros(12,1);

%% VISUALIZATION
% plot the position of joint1 (reachable tragectories of joint 2 and 3 are
% visualized based on this position.)
figure(fig_num)
scatter3(LP.c0(1,1),LP.c0(2,1),LP.c0(3,1) , color, 'DisplayName', robot_name)
str = newline + "  joint1";
text(LP.c0(1,1),LP.c0(2,1),LP.c0(3,1) , str )
hold on;

% visualize the reachable area on the z-xy plane
plot3(POS_e1_tmp1(:,1),POS_e1_tmp1(:,2),POS_e1_tmp1(:,3), color, ... 
      POS_e1_tmp2(:,1),POS_e1_tmp2(:,2),POS_e1_tmp2(:,3), color, ...
      POS_e1_tmp3(:,1),POS_e1_tmp3(:,2),POS_e1_tmp3(:,3), color, ... 
      POS_e1_tmp4(:,1),POS_e1_tmp4(:,2),POS_e1_tmp4(:,3), color, 'DisplayName', robot_name)
% matlab figure settings
grid on;
daspect([1 1 1]) % set grid is standardized
xlabel('x (origin: CoM of robot)')
zlabel('z (origin: CoM of robot)')
xlim([    0 ,  0.5]);
ylim([-0.15 , 0.25]);
legend

hold on;

end

%%% EOF %%%