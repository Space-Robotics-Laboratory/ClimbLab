%%%%%% Update
%%%%%% upd_tumbling_axes
%%%%%% 
%%%%%% Obtain all possible tumbling axes
%%%%%% 
%%%%%% Created 2020-05-05
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-05-06
%
%
% Obtain legs numbers for all possible tumbling axes
%
% Function variables:
%
%     OUTPUT
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)
%         equilibrium_eval_param.tumbling_axes         : Number of legs that makes a tumbling axis (tumbling_axes_numberx2 matrix)
%         equilibrium_eval_param.tumbling_axes_number  : Total number of tumbling axes (scalar)
%     INPUT
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)
%         LP           			 : Link Parameters (SpaceDyn class)
%         SV           			 : State Variables (SpaceDyn class)

function equilibrium_eval_param = upd_tumbling_axes(equilibrium_eval_param, LP, SV)

equilibrium_eval_param.tumbling_axes = [];

cnt = 1;
for a = 1:LP.num_limb
    if SV.sup(a) == 1
        equilibrium_eval_param.tumbling_axes(cnt,1) = a;
        for b = a+1:LP.num_limb
            if SV.sup(b) == 1
                equilibrium_eval_param.tumbling_axes(cnt,2) = b;
                cnt = cnt + 1;
                break
            end
        end
    end
end

% Close polygon with initial support point
equilibrium_eval_param.tumbling_axes(cnt,2) = equilibrium_eval_param.tumbling_axes(1,1);
% Total number of tumbling axes
equilibrium_eval_param.tumbling_axes_number = cnt;

end