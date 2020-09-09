%%%%%% Update
%%%%%% upd_grasp_slip
%%%%%% 
%%%%%% Update gripping and slippage state
%%%%%% 
%%%%%% Created 2020-04-17
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-17
%
%
% Update the real state of the gripper, if it is open or closed, based on the desired state and slippage conditions
%
% Function variables:
%
%     OUTPUT
%         SV                : State variables (SpaceDyn class)
%     INPUT
%         LP                : Link parameter (SpaceDyn class)
%         SV                : State variables (SpaceDyn class)
%         des_SV            : Desired state variables (SpaceDyn class)

function SV = upd_grasp_slip(LP, SV, des_SV)

% Update gripper
SV.sup = des_SV.sup;
% Check Slippage
for i = 1:LP.num_limb
    if SV.slip(i) == 1
        SV.sup(i) = 0;
    end
end

end