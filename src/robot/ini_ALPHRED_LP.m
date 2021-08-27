%%%%%% Initialize
%%%%%% ini_ALPHRED_LP
%%%%%% 
%%%%%% Mass/inertia parameters and link connections for ALPHRED 
%%%%%% 
%%%%%% NOTE: Parameters are based on the following publications: 
%%%%%% [1]  ALPHRED: A Multi-Modal Operations Quadruped Robot for Package Delivery Applications
%%%%%%      https://ieeexplore.ieee.org/document/9134727
%%%%%% [2]  Implementation of a Versatile 3D ZMP Trajectory Optimization Algorithm on a Multi-Modal Legged Robotic Platform
%%%%%%      https://ieeexplore.ieee.org/document/8593968
%%%%%%
%%%%%% Created on      2021-03-09 by Keigo Haji
%%%%%% Last updated on 2021-03-09 by Keigo Haji

function LP = ini_ALPHRED_LP()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% Definition of the robot performance %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% state the type of the joint configuration -> IK analytical solution is switched
LP.joint_allocation_type = 'insect';

% Max. endurable gripping force
LP.F_grip = 0.0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% Definition of each link parameters %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define link connection relationscoxa
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
% Movable limitation of joint <- based on [1]
LP.joint_limit=[-90 90; -90 135; -165 165]; %[deg]

% Distance from base CoM to Coxa joint <- based on [2]
baseCoM2Coxa = 0.08599;     %[m]
% Position vector from base CoM to i-th joint
LP.c0 = [  baseCoM2Coxa/sqrt(2)  0  0 -baseCoM2Coxa/sqrt(2)  0  0 -baseCoM2Coxa/sqrt(2)  0  0  baseCoM2Coxa/sqrt(2)  0  0;
           baseCoM2Coxa/sqrt(2)  0  0  baseCoM2Coxa/sqrt(2)  0  0 -baseCoM2Coxa/sqrt(2)  0  0 -baseCoM2Coxa/sqrt(2)  0  0;
           0.000                 0  0  0.000                 0  0  0.000                 0  0  0.000                 0  0];
       
% Mass of the base (link 0) <- based on [1]
LP.m0 = 13.58; % [kg] 

% Moment of inertia of base <- based on [1]
LP.inertia0 = [ 0.288 0.000 0.000  ; % [kg*m^2] 
                0.000 0.474 0.000  ;
                0.000 0.000 0.287 ];

% Total mass <- based on [1]
LP.mass = 17.55; % [kg]

% Mass & Length & Radius of each link
% For radiuses, there is no spec. in [1][2], but we need the radiuses to
% calculate the inertia tensor of each limb. So, we set the radiuses of
% each link approximately.  
FemurLinkMass = 0.463;      %[kg]   <- based on [1]
FemurLinkLength = 0.350;    %[m]    <- based on [1]
FemurLinkRadius = 0.029;    %[m]    <- approximately

TibiaLinkMass = 0.495;      %[kg]   <- based on [1]
TibiaLinkLength = 0.350;    %[m]    <- based on [1]
TibiaLinkRadius = 0.01167;  %[m]    <- approximately

CoxaLinkMass = (LP.mass - ((FemurLinkMass + TibiaLinkMass)*4 + LP.m0))/4;   %<- approximately
CoxaLinkLength = 0.09778;   %[m]    <- based on [2]
CoxaLinkRadius = 0.70;      %[m]    <- approximately

% Mass of each link 
LP.m = [ CoxaLinkMass FemurLinkMass TibiaLinkMass ...
         CoxaLinkMass FemurLinkMass TibiaLinkMass ...
         CoxaLinkMass FemurLinkMass TibiaLinkMass ...
         CoxaLinkMass FemurLinkMass TibiaLinkMass ]; % [kg]
     
% note that z axis is always rotation axis, and x axis is always towards
% the EE in ClimbLab. So, for the links, the axial direction is x, and 
% Radial directions are y and z.
% For LF(Left Front) limb,
inertiaTensorForLinks.LF.coxa.Ixx = 1/2 * CoxaLinkMass * CoxaLinkRadius^2;
inertiaTensorForLinks.LF.coxa.Iyy = 1/4 * CoxaLinkMass * CoxaLinkRadius^2 + 1/12 * CoxaLinkMass * CoxaLinkLength^2;
inertiaTensorForLinks.LF.coxa.Izz = inertiaTensorForLinks.LF.coxa.Iyy;
inertiaTensorForLinks.LF.femur.Ixx = 1/2 * FemurLinkMass * FemurLinkRadius^2;
inertiaTensorForLinks.LF.femur.Iyy = 1/4 * FemurLinkLength * FemurLinkRadius^2 + 1/12 * FemurLinkLength * FemurLinkLength^2;
inertiaTensorForLinks.LF.femur.Izz = inertiaTensorForLinks.LF.coxa.Iyy;
inertiaTensorForLinks.LF.tibia.Ixx = 1/2 * TibiaLinkMass * TibiaLinkRadius^2;
inertiaTensorForLinks.LF.tibia.Iyy = 1/4 * TibiaLinkMass * TibiaLinkRadius^2 + 1/12 * TibiaLinkMass * TibiaLinkLength^2;
inertiaTensorForLinks.LF.tibia.Izz = inertiaTensorForLinks.LF.coxa.Iyy;
% all limb has the same inertial spec.
inertiaTensorForLinks.LH = inertiaTensorForLinks.LF;
inertiaTensorForLinks.RF = inertiaTensorForLinks.LF;
inertiaTensorForLinks.RH = inertiaTensorForLinks.LF;

% Moment of inertia of each link % [kg*m^2] 
LP.inertia = [ inertiaTensorForLinks.LF.coxa.Ixx 0 0    inertiaTensorForLinks.LF.femur.Ixx 0 0    inertiaTensorForLinks.LF.tibia.Ixx 0 0    inertiaTensorForLinks.LH.coxa.Ixx 0 0   inertiaTensorForLinks.LH.femur.Ixx 0 0    inertiaTensorForLinks.LH.tibia.Ixx 0 0   inertiaTensorForLinks.RH.coxa.Ixx 0 0     inertiaTensorForLinks.RH.femur.Ixx 0 0    inertiaTensorForLinks.RH.tibia.Ixx 0 0    inertiaTensorForLinks.RF.coxa.Ixx 0 0    inertiaTensorForLinks.RF.femur.Ixx 0 0    inertiaTensorForLinks.RF.tibia.Ixx 0 0 ;
               0 inertiaTensorForLinks.LF.coxa.Iyy 0    0 inertiaTensorForLinks.LF.femur.Iyy 0    0 inertiaTensorForLinks.LF.tibia.Iyy 0    0 inertiaTensorForLinks.LH.coxa.Iyy 0   0 inertiaTensorForLinks.LH.femur.Iyy 0    0 inertiaTensorForLinks.LH.tibia.Iyy 0    0 inertiaTensorForLinks.RH.coxa.Iyy 0    0 inertiaTensorForLinks.RH.femur.Iyy 0    0 inertiaTensorForLinks.RH.tibia.Iyy 0    0 inertiaTensorForLinks.RF.coxa.Iyy 0    0 inertiaTensorForLinks.RF.femur.Iyy 0    0 inertiaTensorForLinks.RF.tibia.Iyy 0 ;
               0 0 inertiaTensorForLinks.LF.coxa.Izz    0 0 inertiaTensorForLinks.LF.femur.Izz    0 0 inertiaTensorForLinks.LF.tibia.Izz    0 0 inertiaTensorForLinks.LH.coxa.Izz   0 0 inertiaTensorForLinks.LH.femur.Izz    0 0 inertiaTensorForLinks.LH.tibia.Izz    0 0 inertiaTensorForLinks.RH.coxa.Izz    0 0 inertiaTensorForLinks.RH.femur.Izz    0 0 inertiaTensorForLinks.RH.tibia.Izz    0 0 inertiaTensorForLinks.RF.coxa.Izz    0 0 inertiaTensorForLinks.RF.femur.Izz    0 0 inertiaTensorForLinks.RF.tibia.Izz ];

% Rotational relationscoxa of links frames   
LP.Qi = [  0    pi/2 0  0      pi/2 0  0      pi/2 0 0      pi/2 0;
           0    0    0  0      0    0  0      0    0 0      0    0;
           pi/4 0    0  pi*3/4 0    0  pi*5/4 0    0 pi*7/4 0    0];
       
           
% Position vector from each link CoM to i-th joint
LP.cc = zeros( 3,12,12 ); 

% Postion vector from link 1 CoM to joint 1
LP.cc(:,1,1) = [ -CoxaLinkLength/2  0  0]';
% Postion vector from link 1 CoM to joint 2 
LP.cc(:,1,2) = [  CoxaLinkLength/2    0   0 ]';
LP.cc(:,2,2) = [ -FemurLinkLength/2   0   0 ]';
LP.cc(:,2,3) = [  FemurLinkLength/2   0   0 ]';
LP.cc(:,3,3) = [ -TibiaLinkLength/2   0   0 ]';

LP.cc(:,4,4) = [ -CoxaLinkLength/2  0  0]';
LP.cc(:,4,5) = [  CoxaLinkLength/2    0   0 ]';
LP.cc(:,5,5) = [ -FemurLinkLength/2   0   0 ]';
LP.cc(:,5,6) = [  FemurLinkLength/2   0   0 ]';
LP.cc(:,6,6) = [ -TibiaLinkLength/2   0   0 ]';

LP.cc(:,7,7) = [ -CoxaLinkLength/2  0  0]';
LP.cc(:,7,8) = [  CoxaLinkLength/2    0   0 ]';
LP.cc(:,8,8) = [ -FemurLinkLength/2   0   0 ]';
LP.cc(:,8,9) = [  FemurLinkLength/2   0   0 ]';
LP.cc(:,9,9) = [ -TibiaLinkLength/2   0   0 ]';

LP.cc(:,10,10) = [ -CoxaLinkLength/2  0  0]';
LP.cc(:,10,11) = [  CoxaLinkLength/2    0   0 ]';
LP.cc(:,11,11) = [ -FemurLinkLength/2   0   0 ]';
LP.cc(:,11,12) = [  FemurLinkLength/2   0   0 ]';
LP.cc(:,12,12) = [ -TibiaLinkLength/2   0   0 ]';


% Position vector from end link CoM to end point
footLength = 0.02;  %[m]    <- approximately
TibiaCoM2EE = TibiaLinkLength/2 + footLength;
LP.ce = [  0  0  TibiaCoM2EE    0  0  TibiaCoM2EE   0  0  TibiaCoM2EE   0  0 TibiaCoM2EE ;
           0  0  0              0  0  0             0  0  0             0  0  0    ;
           0  0  0              0  0  0             0  0  0             0  0  0      ];
% Rotational relationscoxa of end link frames
LP.Qe = [  0  0  0  0  0  0  0  0  0  0  0  0;
           0  0  0  0  0  0  0  0  0  0  0  0;
           0  0  0  0  0  0  0  0  0  0  0  0 ];

% EOF