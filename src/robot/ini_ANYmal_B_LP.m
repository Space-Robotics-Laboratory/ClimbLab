%%%%%% Initialize
%%%%%% ini_ANYmal_B_LP
%%%%%% 
%%%%%% Mass/inertia parameters and link connections for ANYmal B
%%%%%% 
%%%%%% NOTE: all parameters are based on the following URDF file
%%%%%% https://github.com/ANYbotics/anymal_b_simple_description
%%%%%% 
%%%%%% Created: 2021-01-18
%%%%%% Kentaro Uno
%%%%%% Last update: 2021-03-03
%%%%%% Kentaro Uno

function LP = ini_ANYmal_B_LP()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% Definition of the robot performance %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% state the type of the joint configuration -> IK analytical solver is switched
LP.joint_allocation_type = 'mammal';
LP.leg_config_type = 'xx';

% Max. endurable gripping force
LP.F_grip = 200.0;

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
% Movable limitation of joint
LP.joint_limit=[-540 540; -540 540; -540 540]; %[deg]

% Position vector from base CoM to i-th joint. Order is LF_HAA, LF_HFE, 
% LF_KFE, LH_HAA, LH_HFE, LH_KFE, RH_HAA, RH_HFE, RH_KFE, RF_HAA, RF_HFE, and RF_KFE.  
LP.c0 = [  0.277  0  0 -0.277  0  0 -0.277  0  0  0.277  0  0;
           0.116  0  0  0.116  0  0 -0.116  0  0 -0.116  0  0;
           0.000  0  0  0.000  0  0  0.000  0  0  0.000  0  0];
% Mass of the base (link 0) <- copied from URDF file 
LP.m0 = 16.793507758; % [kg] 
% Moment of inertia of base <- copied from URDF file
LP.inertia0 = [  0.217391101503   -0.00132873239126 -0.00228200226173  ; % [kg*m^2] 
                -0.00132873239126  0.639432546734   -0.00138078263145  ;
                -0.00228200226173 -0.00138078263145  0.62414077654    ];

% Mass of each link <- based on manual measurement
LP.m = [ 1.42462064 1.634976467 0.207204302 + 0.140170767 ...
         1.42462064 1.634976467 0.207204302 + 0.140170767 ...
         1.42462064 1.634976467 0.207204302 + 0.140170767 ...
         1.42462064 1.634976467 0.207204302 + 0.140170767 ]; % [kg]
% Total mass 
LP.mass = sum(LP.m) + LP.m0; % [kg]

% Rotational relationship of links frames
% local variables to define the offset angle theta_1 and theta_2
LP.theta_1 = 0.0; % [rad]
LP.theta_2 = 0.0; % [rad]

% so, with the rotation about theta_1 around the z axis, then rotation
% about theta_2+pi/2 around the y' axis, the hip joint frame is obtained.
% However, for the SpaceDyn function, we should describe this in not z-y-x
% euler angle, but in x-y-z euler angle!!
LF_euler_angle_ZYX = [LP.theta_1 pi/2+LP.theta_2 0];  LH_euler_angle_ZYX = [-LP.theta_1 pi/2-LP.theta_2 0];
RH_euler_angle_ZYX = [LP.theta_1 pi/2-LP.theta_2 0];  RF_euler_angle_ZYX = [-LP.theta_1 pi/2+LP.theta_2 0];
LF_rotation_matrix = eul2rotm(LF_euler_angle_ZYX,'ZYX');
LH_rotation_matrix = eul2rotm(LH_euler_angle_ZYX,'ZYX');
RH_rotation_matrix = eul2rotm(RH_euler_angle_ZYX,'ZYX');
RF_rotation_matrix = eul2rotm(RF_euler_angle_ZYX,'ZYX');

LF_euler_angle_XYZ = rotm2eul(LF_rotation_matrix,'XYZ');
LH_euler_angle_XYZ = rotm2eul(LH_rotation_matrix,'XYZ');
RH_euler_angle_XYZ = rotm2eul(RH_rotation_matrix,'XYZ');
RF_euler_angle_XYZ = rotm2eul(RF_rotation_matrix,'XYZ');

LP.Qi = [  LF_euler_angle_XYZ(1)  pi/2  0  LH_euler_angle_XYZ(1)   pi/2  0  RH_euler_angle_XYZ(1)  pi/2  0  RF_euler_angle_XYZ(1)   pi/2  0;
           LF_euler_angle_XYZ(2)  0     0  LH_euler_angle_XYZ(2)   0     0  RH_euler_angle_XYZ(2)  0     0  RF_euler_angle_XYZ(2)   0     0;
           LF_euler_angle_XYZ(3)  0     0  LH_euler_angle_XYZ(3)   0     0  RH_euler_angle_XYZ(3)  0     0  RF_euler_angle_XYZ(3)   0     0];

% Moment of inertia of each link <- copied from hubrobo_v3_2_parameters.dat @TODO: current each last link inertial matrixes are replaced with the ones of Tibia links, and the coodinates are based on the URDF ones. These should be adjusted for SpaceDyn one.% [kg*m^2] 
% @TODO: current value is just copied from HubRobo
LP.inertia = [  4.17250591050379E-05 -3.280237461361E-07 -3.28017174197894E-07     1.53134966250451E-05 1.56836205467462E-14 3.77019535347995E-15    2.20258150957412E-05 -3.2869671194301E-07 -3.31326402019975E-07        4.1725E-05 3.3003E-07 -3.3004E-07    1.53134966258197E-05 1.55870791195499E-14 4.18826057628801E-14    2.20258136176618E-05 -3.28696725826472E-07 -3.31325909357267E-07      4.17250600316099E-05 -3.28023694988862E-07 -3.28016569372976E-07    1.53134966250456E-05 1.56835951357578E-14 3.7701969258325E-15     2.20258152259854E-05 -3.28696694818991E-07 -3.31326354131817E-07    4.1725054229097E-05 3.30034360628209E-07 -3.30041495547628E-07    1.531349662582E-05 1.5587029991639E-14 4.18826171023906E-14   4.18826171023906E-14 -1.5341346770838E-14 3.67324197174481E-05;
               -3.280237461361E-07 2.75916233673388E-05 6.28753118367121E-07     1.56836205467462E-14 2.40171123385755E-05 6.33438629632886E-16      -3.2869671194301E-07 1.76456846672338E-05 -3.14370002187497E-07        3.3003E-07 2.7592E-05 -6.1624E-07    1.55870791195499E-14 2.40171123385758E-05 -1.53413473909909E-14   -3.28696725826472E-07 1.76456831216546E-05 -3.14370065697366E-07     -3.28023694988862E-07 2.75916244818459E-05 6.28753097529483E-07    1.56835951357578E-14 2.40171123385754E-05 6.33432116769803E-16     -3.28696694818991E-07 1.76456848327505E-05 -3.14370044226523E-07   3.30034360628209E-07 2.75916215197804E-05 -6.16243771991015E-07    1.5587029991639E-14 2.40171123385757E-05 -1.5341346770838E-14   -3.2869672582661E-07 1.76456831216548E-05 -3.14370065697396E-07;
               -3.28017174197894E-07 6.28753118367121E-07 2.75915908175676E-05     3.77019535347995E-15 6.33438629632886E-16 3.67324197182656E-05    -3.31326402019975E-07 -3.14370002187497E-07 1.19815307988945E-05      -3.3004E-07 -6.1624E-07 2.7592E-05    4.18826057628801E-14 -1.53413473909909E-14 3.67324197174477E-05   -3.31325909357267E-07 -3.14370065697366E-07 1.19815307394935E-05     -3.28016569372976E-07 6.28753097529483E-07 2.75915909936714E-05     3.7701969258325E-15 6.33432116769803E-16 3.67324197182661E-05     -3.31326354131817E-07 -3.1437E-07 1.19815308159254E-05             -3.30041495547628E-07 -6.16243771991015E-07 2.759158797365E-05     4.18826171023906E-14 -1.5341346770838E-14 3.67324197174481E-05   -3.31325909357463E-07 -3.14370065697396E-07 1.19815307394936E-05];

% Position vector from each link CoM to i-th joint
LP.cc = zeros( 3,12,12 ); 

% Postion vector from link 1 CoM to joint 1 seen from the joint 1 frame
LP.cc(:,1,1) = [  0          -0.041/2   -0.0635/2 ]';
% Postion vector from link 1 CoM to joint 2 
LP.cc(:,1,2) = [  0           0.041/2    0.0635/2 ]';
LP.cc(:,2,2) = [ -0.25/2      0          0.109/2 ]';
LP.cc(:,2,3) = [  0.25/2      0         -0.109/2 ]';
LP.cc(:,3,3) = [ -0.32125/2   -0.1/2     0       ]'; 

LP.cc(:,4,4) = [  0          -0.041/2    0.0635/2 ]';
LP.cc(:,4,5) = [  0           0.041/2   -0.0635/2 ]';
LP.cc(:,5,5) = [ -0.25/2      0          0.109/2 ]';
LP.cc(:,5,6) = [  0.25/2      0         -0.109/2 ]';
LP.cc(:,6,6) = [ -0.32125/2   0.1/2      0       ]'; 

LP.cc(:,7,7) = [  0           0.041/2    0.0635/2 ]';
LP.cc(:,7,8) = [  0          -0.041/2   -0.0635/2 ]';
LP.cc(:,8,8) = [ -0.25/2      0         -0.109/2 ]';
LP.cc(:,8,9) = [  0.25/2      0          0.109/2 ]';
LP.cc(:,9,9) = [ -0.32125/2   0.1/2      0       ]'; 

LP.cc(:,10,10) = [  0           0.041/2   -0.0635/2 ]';
LP.cc(:,10,11) = [  0          -0.041/2    0.0635/2 ]';
LP.cc(:,11,11) = [ -0.25/2      0         -0.109/2 ]';
LP.cc(:,11,12) = [  0.25/2      0          0.109/2 ]';
LP.cc(:,12,12) = [ -0.32125/2   -0.1/2     0       ]'; 

% Position vector from end link CoM to end point
LP.ce = [  0  0  0.32125/2 0  0  0.32125/2        0  0  0.32125/2          0  0  0.32125/2  ;
           0  0  0.1/2     0  0 -0.1/2            0  0 -0.1/2              0  0  0.1/2      ;
           0  0  0         0  0  0                0  0  0                  0  0  0         ];
% Rotational relationship of end link frames
LP.Qe = [  0  0  0  0  0  0  0  0  0  0  0  0;
           0  0  0  0  0  0  0  0  0  0  0  0;
           0  0  0  0  0  0  0  0  0  0  0  0 ];

% EOF