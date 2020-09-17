%%%%%%---------------------------------------------------------------------
%%%%%% sample_main.m
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
%%%%%%   ###    @@@@@         09/10/2019
%%%%%%  /-\      @@
%%%%%% |   |        %%        Authors:
%%%%%%  \-/##     %%%%%       Kentaro Uno
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
%%%%%% Two Dimensional Reachable Area Visualization Simulation Code for Leg
%%%%%% Designing
%%%%%% 
%%%%%%---------------------------------------------------------------------
%%%%%% 
%%%%%% Created      2019-09-10 by Kentaro Uno
%%%%%% Last updated 2020-06-21 by Kentaro Uno
%%%%%% 
%%%%%%---------------------------------------------------------------------

clc; clear; close all;
tic

% figure number
fig_num = 1;
addpath(genpath('../..'));
addpath(genpath('../LP_design'));
addpath(genpath('../tools'));

%%
%%%%%%%%%%%%%%%%%%%%%%%%
%%% CLiMBot testbed  %%%
%%%%%%%%%%%%%%%%%%%%%%%%
robot_name = "CLiMBot";
%%% Link parameters loading %%%
LP = LP_to_design_CLiMBot_v3();
%%% initialize state variables %%%
SV = ini_12DOF_SV( LP );

%%% rotate CCW the joint 1 to get along with y axis of the base %%%
%rotAngleForJoint1ForCLiMBot = atan(0.5); % unit: radian
rotAngleForJoint1ForCLiMBot  = 45*pi/180; % unit: radian
joint1 = rotAngleForJoint1ForCLiMBot;
%%% set the limitation of motion of joint2 and 3 %%%
joint2.max = 0;    % unit: degree
joint2.min = -90;     % unit: degree
joint3.max = 135;     % unit: degree
joint3.min = 0;  % unit: degree
%%% set the visualization color %%%
color = 'r';

%%% Reachable area parameter %%%
drawReachableArea( LP, SV, joint1 , joint2 , joint3 , color , fig_num, robot_name );
view(0,0) % view from z axis direction i.e. 2D visualization

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% HubRobo with gripper %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot_name = "HubRobo with gripper";
%%% Link parameters loading %%%
LP = ini_HubRobo_grip_LP();
%%% initialize state variables %%%
SV = ini_12DOF_SV( LP );

%%% rotate CCW the joint 1 to get along with y axis of the base %%%
rotAngleForJoint1ForOldTesbed  = -45*pi/180; % unit: radian
joint1 = rotAngleForJoint1ForOldTesbed;
%%% set the limitation of motion of joint2 and 3 %%%
joint2.max = 90;    % unit: degree
joint2.min = 0;     % unit: degree
joint3.max = 0;     % unit: degree
joint3.min = -135;  % unit: degree
%%% set the visualization color %%%
color = 'b';

%%% Reachable area parameter %%%
drawReachableArea( LP, SV, joint1 , joint2 , joint3 , color , fig_num, robot_name );
view(0,0) % view from z axis direction i.e. 2D visualization


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% HubRobo with link extention %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot_name = "HubRobo new design";
%%% Link parameters loading %%%
LP = LP_to_design_HubRobo();
%%% initialize state variables %%%
SV = ini_12DOF_SV( LP );

%%% rotate CCW the joint 1 to get along with y axis of the base %%%
rotAngleForJoint1ForOldTesbed  = -45*pi/180; % unit: radian
joint1 = rotAngleForJoint1ForOldTesbed;
%%% set the limitation of motion of joint2 and 3 %%%
joint2.max = 90;    % unit: degree
joint2.min = 0;     % unit: degree
joint3.max = 0;     % unit: degree
joint3.min = -135;  % unit: degree
%%% set the visualization color %%%
color = 'g';

%%% Reachable area parameter %%%
drawReachableArea( LP, SV, joint1 , joint2 , joint3 , color , fig_num, robot_name );
view(0,0) % view from z axis direction i.e. 2D visualization


%%% EOF %%%