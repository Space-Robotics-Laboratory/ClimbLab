%%%%%% Update
%%%%%% upd_grasp_detach
%%%%%% 
%%%%%% Update gripping and detachment state
%%%%%% 
%%%%%% Created 2020-04-17
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-10-15 by Kentaro Uno
%
%
% Update the real state of the gripper, if it is open or closed, based on the desired state and detachment conditions
%
% Function variables:
%
%     OUTPUT
%         SV                : State variables (SpaceDyn class)
%     INPUT
%         LP                : Link parameter (SpaceDyn class)
%         SV                : State variables (SpaceDyn class)
%         des_SV            : Desired state variables (SpaceDyn class)

function SV = upd_grasp_detach(LP, SV, des_SV, F_grip, environment_param, equilibrium_eval_param, variables_saved)

% Update gripper
SV.sup = des_SV.sup;
% Check detachment

switch environment_param.detachment_detection_method
    
    case 'max_holding_force'
        
        for i = 1:LP.num_limb
            if variables_saved.(genvarname(['Fe_res' num2str(i)]))(end) > F_grip  &&  SV.Fe(3,3*i) < 0  &&  des_SV.sup(i) == 1
                SV.slip(i) = 1;
            end
        end
        
    case 'tsm'
        if equilibrium_eval_param.tumbling_axes_number > 0
            if equilibrium_eval_param.tsm == 0
                % Leg number for tumbling axis where tumbling happened
                ab = equilibrium_eval_param.tumbling_axes(equilibrium_eval_param.tsm_axes_judgment == 0,:);

                for i = 1:LP.num_limb
                    if SV.sup(i) == 1 && i ~= ab(1) && i ~= ab(2)
                        SV.slip(i) = 1;
                    end
                end
            end
        end
        
    case 'none'
               
end

SV.sup = des_SV.sup;
for i = 1:LP.num_limb
    if SV.slip(i) == 1
        SV.sup(i) = 0;
    end
end

end