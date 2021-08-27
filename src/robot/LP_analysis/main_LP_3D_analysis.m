%%%%%%---------------------------------------------------------------------
%%%%%% main_LP_3D_analysis.m
%%%%%%---------------------------------------------------------------------
%%%%%%           @@
%%%%%%         @@@@@@
%%%%%%        @```@@@@
%%%%%%       @`  `@@@@@@
%%%%%%     @@`   `@@@@@@@@
%%%%%%    @@`    `@@@@@@@@@   Tohoku University
%%%%%%    @` `   `@@@@@@@@@   SPACE ROBOTICS LABORATORY
%%%%%%    @`` ## `@@@@@@@@@   http://www.astro.mech.tohoku.ac.jp/
%%%%%%    @` #..#`@@@@@@@@@
%%%%%%    @` #..#`@@@@@@@@@
%%%%%%    @` ### `@@@@@@@@@   Professor Kazuya Yoshida
%%%%%%    @` ###``@@@@@@@@@   Assistant Professor Kenji Nagaoka
%%%%%%     @### ``@@@@@@@@
%%%%%%     ###`  `@@@@@@@
%%%%%%    ### @`.`@@@@@       Creation Date:
%%%%%%   ###    @@@@@         10/02/2019
%%%%%%  /-\      @@
%%%%%% |   |        %%        Authors:
%%%%%%  \-/##     %%%%%       Kentaro Uno, Naomasa Takada 
%%%%%%    ####  %%%           unoken@astro.mech.tohoku.ac.jp
%%%%%%      ###%%      *
%%%%%%      ##%%     *****
%%%%%%       #%%      ***
%%%%%%         %%     * *
%%%%%%          %%%
%%%%%%            %%%%%
%%%%%%              %%
%%%%%%---------------------------------------------------------------------
%%%%%%
%%%%%% Three Dimensional Reachable Area Visualization Simulation Code for 
%%%%%% Leg Designing
%%%%%% 
%%%%%%---------------------------------------------------------------------
%%%%%% 
%%%%%% Created      2019-09-10 by Kentaro Uno
%%%%%% Last updated 2021-02-02 by Keigo Haji
%%%%%% 
%%%%%%---------------------------------------------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1.The clc function clears all inputs and outputs from the command window
%   display.
% 2.The clear function deletes items from the workspace and releases system
%   memory.
% 3.The close all function deletes all Figures whose handles are not
%   hidden.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear
close all

floor = 0; % height from the floor to the robot base [m] 
inc = 0; % inclination angle of the ground plane [deg/rad?]

%%%%%%%% Defining link parameters and initializing state variables %%%%%%%%
% 1.Acquire the defined link parameters with LP definition file such as LP_climbot_v3().
% 2.Initialize state variables with SV().
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

addpath(genpath('../..'));
addpath(genpath('../LP_to_design'));

%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% Example of old CLiMBot  %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % @TODO: change the LP wrt the new joint allocation
% robot_name = "CLiMBot";
% LP = LP_to_design_CLiMBot_v3();
% LP.num_limb = 4;
% %%% rotate CCW the joint 1 to get along with y axis of the base %%%
% rotAngleForJoint1ForOldTesbed  = 45*pi/180; % unit: radian
% joint1 = rotAngleForJoint1ForOldTesbed;
% %%% set the limitation of motion of joint2 and 3 %%%
% joint2.max = 0;    % unit: degree
% joint2.min = -90;     % unit: degree
% joint3.max = 135;     % unit: degree
% joint3.min = 0;  % unit: degree
% %%% set the visualization color %%%
% color = 'r'
% 
% %%% initialize state variables %%%
% SV = ini_12DOF_SV( LP );

%%

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%% Example of HubRobo      %%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot_name = "HubRobo with gripper";
% LP = ini_HubRobo_grip_LP();
LP = LP_to_design_HubRobo();
LP.num_limb = 4;
%%% rotate CCW the joint 1 to get along with y axis of the base %%%
rotAngleForJoint1ForOldTesbed  = 45*pi/180; % unit: radian
joint1 = rotAngleForJoint1ForOldTesbed;
%%% set the limitation of motion of joint2 and 3 %%%
joint2.max = 90;    % unit: degree
joint2.min = 0;     % unit: degree
joint3.max = 0;     % unit: degree
joint3.min = -135;  % unit: degree
%%% set the visualization color %%%
color = 'r'

%%% initialize state variables %%%
SV = ini_12DOF_SV( LP );

%%

%%%%%%%%%%%%%%%%%%%%%%%% To use defined function %%%%%%%%%%%%%%%%%%%%%%%%%%

% SV = SV( LP );
SV.q = zeros(12,1);

SV.q = [ 0.0; 0.0; -1.5708;
         0.0; 0.0; -1.5708;
         0.0; 0.0; -1.5708;
         0.0; 0.0; -1.5708 ];

SV.q(1) = joint1;
%%% Acquisition of SV.AA
SV = calc_aa( LP, SV );
%%% Acquisition of SV.RR
SV = calc_pos( LP, SV );

%%% Calculate of hand positions (we dont care orientations, which is not 
%%% stored in the following scripts )
[ POS_e(:,1), ORI_e ] = f_kin_e( LP, SV, 3);
[ POS_e(:,2), ORI_e ] = f_kin_e( LP, SV, 6);
[ POS_e(:,3), ORI_e ] = f_kin_e( LP, SV, 9);
[ POS_e(:,4), ORI_e ] = f_kin_e( LP, SV, 12);
%%% Calculate of joints position and posture
[ POS_j, ORI_j ] = f_kin_j( LP, SV, [ 1 2 3 4 5 6 7 8 9 10 11 12 ] );
floor = POS_e(3,1);

%%

%%% set the number of the figure
fig_num = 1;

%%% Use of the defined function
drawRobotWireFrame( LP, SV, floor, POS_e, POS_j, inc, fig_num ); 

%%% visualize the reachable region
addpath(genpath('../tools'));
drawReachableArea( LP, SV, joint1, joint2, joint3, color, fig_num, robot_name );

%%% matlab figure setting
title('Reachable region (origin: CoM of robot)');
xlabel('x  [m]'); 
ylabel('y  [m]'); 
zlabel('z  [m]');
xlim([-0.4  0.4]);
ylim([-0.4  0.4]);
zlim([-0.20  0.2]);
daspect([1 1 1]) % set grid is standardized
grid on;
hold on;
legend("off");


%%% EOF %%%