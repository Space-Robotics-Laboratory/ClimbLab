%%%%%% Calculate
%%%%%% get_pos_i_kine_3dof
%%%%%% 
%%%%%% - Postion Inverse kinematics solver for a 3 DOF manipulator
%%%%%% 
%%%%%% Created 2020-06-25
%%%%%% Kentaro Uno
%%%%%% Last update: 2020-09-22
%
%
% Calculate joints angular position of a 3 degrees of freedom manipulator from the end-effector position
%
% Function variables:
%
%     OUTPUT
%         q_sol     : Initial angular position for joints (1xn vector)
%     INPUT
%         LP        : Link Parameters (SpaceDyn class)
%         SV        : State Variables (SpaceDyn class)
%         POS_e     : Position of the end-effector [m] (3x1 vector)
%         num_e     : Number of the end-effector (scalar)


function q_sol = get_i_kine_3dof( LP, SV, POS_e, num_e )

q_sol = zeros(3,1);
p0e = zeros(3,1); % Base (link 0) to endeffector (link "e") position vector, i.e. input of IK

% num_e = 4;

% for reader of this code, note that the following frame orientation is
% different from the rule of spacedyn. basically no orientation between two
% of coxa, femur, and tibia link.
p01 = zeros(3,1); % Base (link 0) to Coxa (link 1) position vector  
p12 = zeros(3,1); % Coxa (link 1) to Femur (link 2) position vector  
p23 = zeros(3,1); % Femur (link 2) to Tibia (link 3)position vector  
p3e = zeros(3,1); % Tibia (link 3) to endeffector (link "e") position vector  

joints = j_num( LP, num_e );

% p0e should be written seen from the base frame
p0e = POS_e - SV.R0;
p0e = (rot_x(SV.Q0(1,1))*rot_y(SV.Q0(2,1))*rot_z(SV.Q0(3,1))*p0e);
POS_eE = p0e;
% p0e = [0.15;0.10;-0.13];
% load link parameter: LP file and substituting into the local constant 
p01(:,1) = LP.c0(:, joints(1));
p12(:,1) = LP.cc(:, joints(1)  , joints(1)+1) - LP.cc(:,joints(1)  , joints(1)  );
p23_tmp(:,1) = LP.cc(:, joints(1)+1, joints(1)+2) - LP.cc(:,joints(1)+1, joints(1)+1);
% adjusting the frame for IK from frame of SpaceDyn
p23(1,1) = p23_tmp(1,1); p23(2,1) = -p23_tmp(3,1); p23(3,1) = p23_tmp(2,1);
p3e_tmp(:,1) = LP.ce(:, joints(1)+2) - LP.cc(:,joints(1)+2, joints(1)+2);
% adjusting the frame for IK from frame of SpaceDyn
p3e(1,1) = p3e_tmp(2,1); p3e(2,1) = -p3e_tmp(3,1); p3e(3,1) = -p3e_tmp(1,1);

%%% joint base to Coxa solver 
% yaw rotation of Base_to_Coxa joint seen from basve frame
alpha = LP.Qi(3,joints(1));
% a1, b1, c1: temporary valuabled to calculate q_sol 1
a1 = - p0e(1,1) + p01(1,1);
b1 =   p0e(2,1) - p01(2,1);
c1 =   p12(2,1) + p23(2,1) + p3e(2,1);

q1 = atan2( a1 , b1 ) + atan2( sqrt(a1^2+b1^2-c1^2), c1 ) - alpha;

%%% joint Coxa to Femur solver 
% A, B, a2, b2, c2: temporary valuabled to calculate q_sol 1
A = (p0e(1,1)-p01(1,1))*cos(alpha+q1) + (p0e(2,1) - p01(2,1))*sin(alpha+q1) - p12(1,1);
B = p0e(3,1) - p01(3,1) - p12(3,1);
a2 = 2*(B*p23(1,1)+A*p23(3,1));
b2 = 2*(A*p23(1,1)+B*p23(3,1));
c2 = A^2 + B^2 + p23(1,1)^2 + p23(3,1)^2 - p3e(1,1)^2 - p3e(3,1)^2;

q2 = - ( atan2( a2 , b2 ) + atan2( sqrt(a2^2+b2^2-c2^2), c2 ) );

%%% joint Femur to Tibia solver
q3 = 2*pi() + atan2( A*cos(q2)-B*sin(q2)-p23(1,1) , A*sin(q2)+B*cos(q2)-p23(3,1) ) - atan2(p3e(1,1), p3e(3,1));

q_sol(1,1) = q1;
q_sol(2,1) = - q2;
q_sol(3,1) = - ( q3 + pi()/2 );

%%% adjust the solutions to be described radians from -2*pi to 2*pi()
if q_sol(1,1) > pi()
    q_sol(1,1) = q_sol(1,1) - 2*pi();
elseif q_sol(1,1) < -pi()
    q_sol(1,1) = q_sol(1,1) + 2*pi();
end

if q_sol(2,1) > pi()
    q_sol(2,1) = q_sol(2,1) - 2*pi();
elseif q_sol(2,1) < -pi()
    q_sol(2,1) = q_sol(2,1) + 2*pi();
end 

if q_sol(3,1) > pi()
    q_sol(3,1) = q_sol(3,1) - 2*pi();
elseif q_sol(3,1) < -pi()
    q_sol(3,1) = q_sol(3,1) + 2*pi();
end 

end

