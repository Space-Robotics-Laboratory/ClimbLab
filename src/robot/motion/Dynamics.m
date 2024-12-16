classdef Dynamics < handle
  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    kUseDynamics_ (1, 1) logical;
  end

  %% Public Methods
  methods (Access = public)

    function dynamics = Dynamics(use_dynamics)
    % Dynamics() Constructor
      arguments (Input)
        use_dynamics (1, 1) {mustBeA(use_dynamics, "logical")};
      end
      dynamics.kUseDynamics_ = use_dynamics;
    end

    function robot = computeForward(dynamics, robot)
    % computeForward()
    %   Compute forward dynamics using SpaceDyn functions mainly "f_dyn_rk2()" and "f_dyn()"
      arguments (Input)
        dynamics;
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      LP_tmp = robot.LP_.clone();
      SV_tmp = robot.SV_.clone();

      if (dynamics.kUseDynamics_)  % Dynamics on
        % Solve equation of motion
        SV_tmp = f_dyn_rk2(LP_tmp, SV_tmp);
        SV_tmp = f_dyn(LP_tmp, SV_tmp);

      else  % Dynamics off
        des_SV_tmp = robot.des_SV_.clone();
        SV_tmp = des_SV_tmp;
      end

      % Calculate links orientations, positions, velocities and accelerations
      SV_tmp = calc_aa(LP_tmp, SV_tmp);
      SV_tmp = calc_pos(LP_tmp, SV_tmp);
      SV_tmp = calc_vel(LP_tmp, SV_tmp);
      SV_tmp = calc_acc(LP_tmp, SV_tmp);

      robot.overwriteStateVariables(SV_tmp);
    end

  end

end  % Dynamics
