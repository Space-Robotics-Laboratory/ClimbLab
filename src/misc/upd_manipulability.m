%%%%%% Update
%%%%%% upd_manipulability
%%%%%% 
%%%%%% calculate manipulability
%%%%%% 
%%%%%% Created 2020-04-10
%%%%%% Koizumi Yusuke
%%%%%% Last update: 2020-10-30
%
%
% Calculate manipulability of each limb
%
%     Function variables:
%
%     OUTPUT
%         evaluation_param : Parameters for evaluation (struct)
%     INPUT
%         LP               : Link Parameters
%         SV               : State Variables
%         evaluation_param : Parameters for evaluation (struct)


function [ evaluation_param ] = upd_manipulability(LP,SV,evaluation_param)
mani = zeros(1,LP.num_limb);
joints = zeros(LP.num_limb,3);
for i = 1:LP.num_limb
    joints(i,:) = j_num(LP, i);
    je = calc_je(LP,SV,joints(i,:));
    mani(i) = sqrt(det(je(1:3,3*i-2:3*i)*je(1:3,3*i-2:3*i)'));
end
evaluation_param.manipulability = mani;
end