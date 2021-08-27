%%%%%% Calculate
%%%%%% get_i_kine_mammal_config_3dof_limb.m
%%%%%% 
%%%%%% - Position inverse kinematics solver for a 3 DOF manipulator that
%%%%%% has a mammalian configuration
%%%%%% 
%%%%%% Created: 2021-01-18
%%%%%% Kentaro Uno
%%%%%% Last update: 2021-02-12
%%%%%% Kentaro Uno
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


function q_sol = get_i_kine_mammal_config_3dof_limb( LP, SV, POS_e, num_e )

%% variables declearation
q_sol = zeros(3,1);
p0e = zeros(3,1); % Base (link 0) to endeffector (link "e") position vector, i.e. input of IK

%% link parameters substitiono into the local variables to solve the IK
% for reader of this code, note that the following frame orientation is
% different from the rule of spacedyn. basically no orientation between two
% of Hip, Thigh, and Shank link.
p01 = zeros(3,1); % Base  (link 0) to Hip   (link 1) position vector  
p12 = zeros(3,1); % Hip   (link 1) to Thigh (link 2) position vector  
p23 = zeros(3,1); % Thigh (link 2) to Shank (link 3) position vector  
p3e = zeros(3,1); % Shank (link 3) to endeffector (link "e") position vector  

joints = j_num( LP, num_e );

% p0e should be written seen from the base frame
p0e = POS_e - SV.R0;
p0e = SV.A0'*p0e;
POS_eE = p0e;

% load link parameter: LP file and substituting into the local constant 
p01(:,1) = LP.c0(:, joints(1));
p12_tmp(:,1) = LP.cc(:, joints(1)  , joints(1)+1) - LP.cc(:,joints(1)  , joints(1)  );
p23_tmp(:,1) = LP.cc(:, joints(1)+1, joints(1)+2) - LP.cc(:,joints(1)+1, joints(1)+1);
p3e_tmp(:,1) = LP.ce(:, joints(1)+2) - LP.cc(:,joints(1)+2, joints(1)+2);

% adjusting the frame for IK from frame of SpaceDyn
p12(1,1) = p12_tmp(3,1); p12(2,1) =  p12_tmp(2,1); p12(3,1) = -p12_tmp(1,1);
p23(1,1) = p23_tmp(2,1); p23(2,1) = -p23_tmp(3,1); p23(3,1) = -p23_tmp(1,1);
p3e(1,1) = p3e_tmp(2,1); p3e(2,1) = -p3e_tmp(3,1); p3e(3,1) = -p3e_tmp(1,1);

% extract the offset angle of the hip joint 
if num_e == 1 || num_e == 3
    theta_1 = LP.theta_1;
else
    theta_1 = - LP.theta_1;
end     
if num_e == 1 || num_e == 4
    theta_2 = LP.theta_2;
else
    theta_2 = - LP.theta_2;
end

%% joint base to Hip solver 
% a1, b1, c1: temporary variable to calculate q1
a1 =    cos(theta_1) * sin(theta_2) * (p0e(1,1)-p01(1,1))...
      + sin(theta_1) * sin(theta_2) * (p0e(2,1)-p01(2,1))...
      +                cos(theta_2) * (p0e(3,1)-p01(3,1));
b1 =  - sin(theta_1)*(p0e(1,1)-p01(1,1))... 
      + cos(theta_1)*(p0e(2,1)-p01(2,1));
c1 = p12(2,1) + p23(2,1) + p3e(2,1);

q1 = atan2( a1 , b1 ) + atan2( sqrt(a1^2+b1^2-c1^2), c1 );

%% joint Hip to Thigh solver 
% A, B, a2, b2, c2: temporary variable to calculate q2
A =    cos(theta_1) * cos(theta_2) * ( p0e(1,1) - p01(1,1) )...
     + sin(theta_1) * cos(theta_2) * ( p0e(2,1) - p01(2,1) )...
     -                sin(theta_2) * ( p0e(3,1) - p01(3,1) )...
     - p12(1,1);
B =    (  sin(theta_1)*sin(q1) + cos(theta_1)*sin(theta_2)*cos(q1) ) * ( p0e(1,1) - p01(1,1) )...
     + ( -cos(theta_1)*sin(q1) + sin(theta_1)*sin(theta_2)*cos(q1) ) * ( p0e(2,1) - p01(2,1) )...
     +                                        cos(theta_2) * cos(q1) * ( p0e(3,1) - p01(3,1) )...
     - p12(3,1);
a2 = 2*(A*p23(3,1)-B*p23(1,1));
b2 = 2*(A*p23(1,1)+B*p23(3,1));
c2 = A^2 + B^2 + p23(1,1)^2 + p23(3,1)^2 - p3e(1,1)^2 - p3e(3,1)^2;

% If we want xx config solution, the sign of the hip to thigh joint solution
% should be flipped between front legs and hind legs.

switch LP.leg_config_type
    case 'oo'
        if num_e == 1 || num_e == 4
            q2 = atan2( a2 , b2 ) - atan2( sqrt(a2^2+b2^2-c2^2), c2 ); % oo config
        elseif num_e == 2 || num_e == 3
            q2 = atan2( a2 , b2 ) + atan2( sqrt(a2^2+b2^2-c2^2), c2 ); % oo config
        end
    case 'xx' 
        if num_e == 1 || num_e == 4
            q2 = atan2( a2 , b2 ) + atan2( sqrt(a2^2+b2^2-c2^2), c2 ); % xx config
        elseif num_e == 2 || num_e == 3
            q2 = atan2( a2 , b2 ) - atan2( sqrt(a2^2+b2^2-c2^2), c2 ); % xx config
        end
    case 'M'
        if num_e == 1 || num_e == 4
            q2 = atan2( a2 , b2 ) + atan2( sqrt(a2^2+b2^2-c2^2), c2 ); 
        elseif num_e == 2 || num_e == 3
            q2 = atan2( a2 , b2 ) + atan2( sqrt(a2^2+b2^2-c2^2), c2 ); 
        end                
end

% % If theta2 is negative, it is forced to be "oo" config
% if LP.theta_2 < 0
%     if num_e == 1 || num_e == 4
%         q2 = atan2( a2 , b2 ) - atan2( sqrt(a2^2+b2^2-c2^2), c2 ); 
%     elseif num_e == 2 || num_e == 3
%         q2 = atan2( a2 , b2 ) + atan2( sqrt(a2^2+b2^2-c2^2), c2 ); 
%     end 
% end

%% joint Thigh to Shank solver
q3 = 2*pi() + atan2( A*cos(q2)-B*sin(q2)-p23(1,1) , A*sin(q2)+B*cos(q2)-p23(3,1) ) - atan2(p3e(1,1), p3e(3,1));

%% fit back the range of the joint into the spacedyn solver 
q_sol(1,1) = q1;
q_sol(2,1) = - q2;
q_sol(3,1) = - q3;

%% adjust the solutions to be described radians from -pi to pi()
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

