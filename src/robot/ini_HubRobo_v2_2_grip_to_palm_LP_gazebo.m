%%%%%% Initialize
%%%%%% ini_HubRobo_v2_2_grip_to_palm_LP
%%%%%% 
%%%%%% Mass/inertia parameters and link connections for HubRobo with grippers
%%%%%% Last link length is defined as the distance from
%%%%%% joint 3 (joint_Femur_to_Tibia) to the center of palm of gripper
%%%%%% 
%%%%%% Created on      2018-02-23 by Warley Ribeiro
%%%%%% Last updated on 2020-06-21 by Kentaro Uno

function LP = ini_HubRobo_v2_2_grip_to_palm_LP()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% Definition of each link parameters %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% state the type of the joint configuration -> IK analytical solution is switched
LP.leg_config_type = 'insect';

% Define link connection relationship
LP.BB = [ 0 1 2 0 4 5 0 7 8 0 10 11];

% Number of links/joints
LP.num_q = length(LP.BB); 

LP.SS = [ -1  1  0  0  0  0  0  0  0  0  0  0;
           0 -1  1  0  0  0  0  0  0  0  0  0;
           0  0 -1  0  0  0  0  0  0  0  0  0;
           0  0  0 -1  1  0  0  0  0  0  0  0;
           0  0  0  0 -1  1  0  0  0  0  0  0;
           0  0  0  0  0 -1  0  0  0  0  0  0;
           0  0  0  0  0  0 -1  1  0  0  0  0;
           0  0  0  0  0  0  0 -1  1  0  0  0;
           0  0  0  0  0  0  0  0 -1  0  0  0;
           0  0  0  0  0  0  0  0  0 -1  1  0;
           0  0  0  0  0  0  0  0  0  0 -1  1;
           0  0  0  0  0  0  0  0  0  0  0 -1];
% Links connected to base (link 0)
LP.S0 = [ 1 0 0 1 0 0 1 0 0 1 0 0];
% End link (leg)
LP.SE = [ 0 0 1 0 0 1 0 0 1 0 0 1 ];
% Type of joint
LP.J_type = [ 'R' 'R' 'R'  'R' 'R' 'R'  'R' 'R' 'R'  'R' 'R' 'R' ];

% Position vector from base CoM to i-th joint
LP.c0 = [  0.054  0  0 -0.054  0  0 -0.054  0  0  0.054  0  0;
           0.054  0  0  0.054  0  0 -0.054  0  0 -0.054  0  0;
           0.000  0  0  0.000  0  0  0.000  0  0  0.000  0  0];
% Mass of the base (link 0)
% LP.m0 = 0.6762;
LP.m0 = 0.8502; % New measurement with upboard
% Moment of inertia of base
LP.inertia0 = [ 62404194*10^-11  0               0 ;
                0               62524799*10^-11  0 ;
                0               0               116269155*10^-11];

% Mass of each link
LP.m = [ 0.0945 0.1104 0.0088 0.0945 0.1104 0.0088 0.0945 0.1104 0.0088 ...
         0.0945 0.1104 0.0088 ];
% Total mass 
LP.mass = sum(LP.m) + LP.m0;
% Rotational relationship of links frames
LP.Qi = [  0    pi/2 0  0      pi/2 0  0      pi/2 0 0      pi/2 0;
           0    0    0  0      0    0  0      0    0 0      0    0;
           pi/4 0    0  pi*3/4 0    0  pi*5/4 0    0 pi*7/4 0    0];
% Moment of inertia of each link
LP.inertia = [  1.686642046585767e-06  0                      0                      4.271415082850776e-05  0                      0                      4.372178464980286e-06  0                      0                        1.686642046585767e-06  0                      0                      4.271415082850776e-05  0                      0                      4.372178464980286e-06  0                      0                        1.686642046585767e-06  0                      0                      4.271415082850776e-05  0                      0                      4.372178464980286e-06  0                      0                        1.686642046585767e-06  0                      0                      4.271415082850776e-05  0                      0                      4.372178464980286e-06  0                      0                      ;
                0                      1.686642046585767e-06  0                      0                      4.271415082850776e-05  0                      0                      4.372178464980286e-06  0                        0                      1.686642046585767e-06  0                      0                      4.271415082850776e-05  0                      0                      4.372178464980286e-06  0                        0                      1.686642046585767e-06  0                      0                      4.271415082850776e-05  0                      0                      4.372178464980286e-06  0                        0                      1.686642046585767e-06  0                      0                      4.271415082850776e-05  0                      0                      4.372178464980286e-06  0                      ; 
                0                      0                      4.287324265048666e-07  0                      0                      2.162286114968218e-05  0                      0                      1.610569299605712e-07    0                      0                      4.287324265048666e-07  0                      0                      2.162286114968218e-05  0                      0                      1.610569299605712e-07    0                      0                      4.287324265048666e-07  0                      0                      2.162286114968218e-05  0                      0                      1.610569299605712e-07    0                      0                      4.287324265048666e-07  0                      0                      2.162286114968218e-05  0                      0                      1.610569299605712e-07  ];  

% Position vector from each link CoM to i-th joint
LP.cc = zeros( 3,12,12 ); 

% Postion vector from link 1 CoM to joint 1
LP.cc(:,1,1) = [ -0.014224  -0.0001013  -0.0085025 ]';
% Postion vector from link 1 CoM to joint 2 
LP.cc(:,1,2) = [  0.014224 0.0001013  0.0085025 ]';
LP.cc(:,2,2) = [ -0.0285685 0  0 ]';
LP.cc(:,2,3) = [  0.0285685 0  0 ]';
LP.cc(:,3,3) = [ -0.0738 0  0 ]'; 

LP.cc(:,4,4) = [ -0.014224  -0.0001013  -0.0085025 ]'; 
LP.cc(:,4,5) = [  0.014224 0.0001013  0.0085025 ]'; 
LP.cc(:,5,5) = [ -0.0285685 0  0  ]';
LP.cc(:,5,6) = [  0.0285685 0  0 ]';
LP.cc(:,6,6) = [ -0.0738 0  0 ]';

LP.cc(:,7,7) = [ -0.014224  -0.0001013  -0.0085025 ]';
LP.cc(:,7,8) = [  0.014224 0.0001013  0.0085025 ]';
LP.cc(:,8,8) = [ -0.0285685 0  0  ]';
LP.cc(:,8,9) = [  0.0285685 0  0 ]';
LP.cc(:,9,9) = [ -0.0738 0  0 ]';

LP.cc(:,10,10) = [ -0.014224  -0.0001013  -0.0085025 ]';
LP.cc(:,10,11) = [  0.014224 0.0001013  0.0085025 ]';
LP.cc(:,11,11) = [ -0.0285685 0  0  ]';
LP.cc(:,11,12) = [  0.0285685 0  0 ]';
LP.cc(:,12,12) = [ -0.0738 0  0 ]';

% Position vector from end link CoM to end point
LP.ce = [  0  0  0.0738   0  0  0.0738   0  0  0.0738   0  0  0.0738 ;
           0  0  0        0  0  0        0  0  0        0  0  0      ;
           0  0  0        0  0  0        0  0  0        0  0  0      ];
% Rotational relationship of end link frames
LP.Qe = [  0  0  0  0  0  0  0  0  0  0  0  0;
           0  0  0  0  0  0  0  0  0  0  0  0;
           0  0  0  0  0  0  0  0  0  0  0  0];

% EOF