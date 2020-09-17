%%%%%% Initialize
%%%%%% ini_12DOF_SV
%%%%%% 
%%%%%% Initialize state variables of a 12 degrees-of-freedom robot
%%%%%% 
%%%%%% Created 2018-02-23
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-02-25

function SV = ini_12DOF_SV( ~ ) 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% Variables initialization %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

SV.q = zeros( 12,1 );   % Joint angle
SV.qd  = zeros( 12,1 ); % Joint angular velocity 
SV.qdd = zeros( 12,1 ); % Joint angular acceleration

SV.v0  = [ 0 0 0 ]'; % Base linear velocity 
SV.w0  = [ 0 0 0 ]'; % Base angular velocity
SV.vd0 = [ 0 0 0 ]'; % Base linear acceleration
SV.wd0 = [ 0 0 0 ]'; % Base angular acceleration

SV.vv = zeros( 3,12 ); % Linear velocity of each link CoM
SV.ww = zeros( 3,12 ); % Angular velocity of each link CoM 
SV.vd = zeros( 3,12 ); % Linear acceleration of each link CoM
SV.wd = zeros( 3,12 ); % Angular acceleration of each link CoM

SV.R0 = [ 0 0 0 ]'; % Base position
SV.Q0 = [ 0 0 0 ]'; % Base orientation (Euler)
SV.A0 = eye(3);     % Base orientation (DCM)

SV.Fe = zeros( 3,12 ); % External force applied to end point 
SV.Te = zeros( 3,12 ); % External torque applied to end point
SV.F0 = [ 0 0 0 ]';    % External force applied to base
SV.T0 = [ 0 0 0 ]';    % External torque applied to base 

SV.tau =zeros( 12,1 ); % Torque applied to each joint

%%% EOF