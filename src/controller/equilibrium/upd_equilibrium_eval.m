%%%%%% Update
%%%%%% upd_equilibrium_eval
%%%%%% 
%%%%%% Update equilibrium evaluation
%%%%%% 
%%%%%% Created 2020-04-23
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-02
%
%
% Update equilibrium parameter evaluation
%
% Function variables:
%
%     OUTPUT
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)
%     INPUT
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)
%         surface_param          : Parameters for surface (class)
%		  LP                     : Link parameters (SpaceDyn class)
%         SV 	      			 : State variables (SpaceDyn class)
%         POS_e      		     : Position of the end-effector [m] (3xnum_limb matrix)
%         F_grip       			 : Maximum gripping force [N] (scalar)


function equilibrium_eval_param = upd_equilibrium_eval(equilibrium_eval_param, surface_param, LP, SV, POS_e, F_grip)


switch equilibrium_eval_param.type


case 'none'

case 'tsm'
	%%% Calculate force and moment acting on the robot gravitational terms
	% Calculate Center of Mass (CoM)
	CoM = upd_CoM(LP, SV);
	% Calculate gravitational terms
	equilibrium_eval_param = upd_gravitational_force_moment(equilibrium_eval_param, LP, CoM);
	% Calculate acceleration terms
	equilibrium_eval_param = upd_inertial_force_moment_plus_rotational(equilibrium_eval_param, LP, SV);

	%%% Obtain Tumbling axes
	equilibrium_eval_param = upd_tumbling_axes(equilibrium_eval_param, LP, SV);
	
	%%% Tumble stability
	% Tumbling moment Mab
	equilibrium_eval_param = upd_tsm_tumbling_moment(equilibrium_eval_param, LP, SV, POS_e, F_grip);
	% Tumbling criterion
	equilibrium_eval_param = upd_tsm_equilibrium_judgment(equilibrium_eval_param, LP, SV, POS_e);
	% TSM
	equilibrium_eval_param = upd_tsm(equilibrium_eval_param, LP);

	%%% Equilibrium flag
	equilibrium_eval_param.equilibrium_flag = equilibrium_eval_param.tsm_equilibrium_flag;

case 'gia'
	% Calculate Center of Mass (CoM)
	CoM = upd_CoM(LP, SV);
	% Calculate acceleration terms
	equilibrium_eval_param = upd_inertial_force_moment_plus_rotational(equilibrium_eval_param, LP, SV);
    % Center of Gravity acceleration
    a_g = equilibrium_eval_param.Fa/LP.mass;
    % Robot mass
    mass = LP.mass;
    % grasp flag
    grasp_flag = SV.sup;

    % External Force and Moment
    F0 = [0 0 0]';
    M0 = [0 0 0]';

    floor_base = surface_param.min;
    
    if strcmp(equilibrium_eval_param.plot_polyhedron,'on')
        plot_on = 1;
    else
        plot_on = 0;
    end

    % To show stability polyhedron in cartesian space shrinked by a factor of "expansion_factor"
    expansion_factor = equilibrium_eval_param.expansion_factor;
    
    [polyhedron, gia, equ_flag] = equ_gia_polyhedron_calc(POS_e, CoM, a_g, mass, grasp_flag, F_grip, F0, M0, plot_on, ...
                                                      floor_base, expansion_factor);

    % Acceleration Margin
    [acc_margin, acc_margin_ab] = equ_gia_acceleration_margin(polyhedron, gia, equ_flag);
    % Inclination Margin
	[inclination_margin, inclination_margin_ab] = equ_gia_inclination_margin(polyhedron, gia, equ_flag);
    
	equilibrium_eval_param.polyhedron = polyhedron;
    equilibrium_eval_param.gia = gia;
    equilibrium_eval_param.equilibrium_flag = equ_flag;
    equilibrium_eval_param.acc_margin = acc_margin;
    equilibrium_eval_param.acc_margin_ab = acc_margin_ab;
    equilibrium_eval_param.inclination_margin = inclination_margin;
    equilibrium_eval_param.inclination_margin_ab = inclination_margin_ab;

otherwise
	
	disp('Invalid equilibrium evaluation type')

end

end