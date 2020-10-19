%%%%%% Initialize
%%%%%% ini_HubRobo_v3_1_grip_to_palm_LP
%%%%%% 
%%%%%% Mass/inertia parameters and link connections for HubRobo with grippers
%%%%%% Last link length is defined as the distance from
%%%%%% joint 3 (joint_Femur_to_Tibia) to the center of palm of gripper
%%%%%% 
%%%%%% Created on      2018-08-25 by Kentaro Uno
%%%%%% Last updated on 2020-08-25 by Kentaro Uno

function LP = ini_HubRobo_v3_1_grip_to_palm_LP()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% Definition of each link parameters %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
% Mass of the base (link 0) <- copied from CAD file
LP.m0 = 0.528; % [kg]
% Moment of inertia of base <- copied from CAD file
LP.inertia0 = [  0.00139923927   -0.00005587288   0.00001742671 ; % [kg*m^2] 
                -0.00005587288    0.00092121743   0.0001742438  ;
                 0.00001742671    0.0001742438    0.00121641609];

% Mass of each link <- based on manual measurement
LP.m = [ 0.187 0.046 0.211 0.187 0.046 0.211 0.187 0.046 0.211 ...
         0.187 0.046 0.211 ]; % [kg]
% Total mass 
LP.mass = sum(LP.m) + LP.m0; % [kg]
% Rotational relationship of links frames
LP.Qi = [  0    pi/2 0  0      pi/2 0  0      pi/2 0 0      pi/2 0;
           0    0    0  0      0    0  0      0    0 0      0    0;
           pi/4 0    0  pi*3/4 0    0  pi*5/4 0    0 pi*7/4 0    0];
% Moment of inertia of each link <- copied from URDF % [kg*m^2] 
LP.inertia = [  4.1697e-05   -3.2802e-07   -3.2802e-07     3.3705e-05            0            0    4.372178464980286e-06                        0                         0       4.1697e-05    3.3003E-07   -3.3004E-07    3.3705e-05            0            0    4.372178464980286e-06                        0                         0      4.1697e-05   -3.2802e-07   -3.2802e-07     3.3705e-05            0            0    4.372178464980286e-06                        0                         0       4.1697e-05    3.3003E-07   -3.3004E-07    3.3705e-05            0            0    4.372178464980286e-06                        0                         0;
               -3.2802e-07    2.7552e-05    6.2875e-07     0             2.204e-05            0                        0    4.372178464980286e-06                         0       3.3003E-07    2.7552e-05   -6.1624E-07    0             2.204e-05            0                        0    4.372178464980286e-06                         0     -3.2802e-07    2.7552e-05    6.2875e-07     0             2.204e-05            0                        0    4.372178464980286e-06                         0       3.3003E-07    2.7552e-05   -6.1624E-07    0             2.204e-05            0                        0    4.372178464980286e-06                         0;
               -3.2802e-07    6.2875e-07    2.7552e-05     0                     0   4.7696e-05                        0                        0     1.610569299605712e-07      -3.3004E-07   -6.1624E-07    2.7552e-05    0                     0   4.7696e-05                        0                        0     1.610569299605712e-07     -3.2802e-07    6.2875e-07    2.7552e-05     0                     0   4.7696e-05                        0                        0     1.610569299605712e-07      -3.3004E-07   -6.1624E-07    2.7552e-05    0                     0   4.7696e-05                        0                        0     1.610569299605712e-07];

% Position vector from each link CoM to i-th joint
LP.cc = zeros( 3,12,12 ); 

% Postion vector from link 1 CoM to joint 1
LP.cc(:,1,1) = [ -0.0142  0  -0.00875 ]';
% Postion vector from link 1 CoM to joint 2 
LP.cc(:,1,2) = [  0.0142 0  0.00875 ]';
LP.cc(:,2,2) = [ -0.0535 0  0 ]';
LP.cc(:,2,3) = [  0.0535 0  0 ]';
LP.cc(:,3,3) = [ -0.0730 0  0 ]'; 

LP.cc(:,4,4) = [ -0.0142 0  -0.00875 ]'; 
LP.cc(:,4,5) = [  0.0142 0   0.00875 ]'; 
LP.cc(:,5,5) = [ -0.0535 0  0 ]';
LP.cc(:,5,6) = [  0.0535 0  0 ]';
LP.cc(:,6,6) = [ -0.0730 0  0 ]';

LP.cc(:,7,7) = [ -0.0142 0  -0.00875 ]';
LP.cc(:,7,8) = [  0.0142 0   0.00875 ]';
LP.cc(:,8,8) = [ -0.0535 0  0 ]';
LP.cc(:,8,9) = [  0.0535 0  0 ]';
LP.cc(:,9,9) = [ -0.0730 0  0 ]';

LP.cc(:,10,10) = [ -0.0142 0  -0.00875 ]';
LP.cc(:,10,11) = [  0.0142 0   0.00875 ]';
LP.cc(:,11,11) = [ -0.0535 0  0 ]';
LP.cc(:,11,12) = [  0.0535 0  0 ]';
LP.cc(:,12,12) = [ -0.0730 0  0 ]';

% Position vector from end link CoM to end point
LP.ce = [  0  0  0.0730   0  0  0.0730   0  0  0.0730   0  0  0.0730 ;
           0  0  0        0  0  0        0  0  0        0  0  0      ;
           0  0  0        0  0  0        0  0  0        0  0  0      ];
% Rotational relationship of end link frames
LP.Qe = [  0  0  0  0  0  0  0  0  0  0  0  0;
           0  0  0  0  0  0  0  0  0  0  0  0;
           0  0  0  0  0  0  0  0  0  0  0  0];

% EOF