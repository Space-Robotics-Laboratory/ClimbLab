classdef DataLogger < dynamicprops & handle
% Log data
% NOTE: DataLogger takes over properties of ConfigSaveSettings. Please refer to ConfigSaveSettings.
%
% Created     : 2020.05.12 by Warley Ribeiro
% Last updated: 2025.01.05 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    index_ (1, 1) double;  % int
    variables_log_ (1, 1) struct;
  end
  properties (Access = private)
    kSaveDataDirName_ (1, 1) string;
  end

  %% Public Methods
  methods (Access = public)

    function data_logger = DataLogger(config_save_settings, run_cod, run_id)
    % Constructor
      arguments (Input)
        config_save_settings (1, 1) {mustBeA(config_save_settings, "ConfigSaveSettings")};
        run_cod (1, 1) {mustBeA(run_cod, "string")};
        run_id  (1, 1) {mustBeA(run_id,  "string")};
      end

      % Add properties from ConfigSaveSettings
      kConfigPropName = properties(config_save_settings);
      for i = 1 : length(kConfigPropName)
        dynamic_property = addprop(data_logger, kConfigPropName{i, 1});
        dynamic_property.Access = "private";
        data_logger.(kConfigPropName{i, 1}) = config_save_settings.(kConfigPropName{i, 1});
      end

      data_logger.index_ = 0;
      data_logger.variables_log_ = struct;

      data_logger.kSaveDataDirName_ = "dat" + filesep + run_cod + filesep + run_id;
      if ((data_logger.kSaveCsvFile_ || data_logger.kSaveConfigFile_) && ...
          ~isfolder(data_logger.kSaveDataDirName_))
        mkdir(data_logger.kSaveDataDirName_);
      end
    end

    function saveVariables(data_logger, time, robot, evaluation)
    % Save variables
      arguments (Input)
        data_logger;
        time       (1, 1) {mustBeA(time,  "double")};
        robot      (1, 1) {mustBeA(robot, "Robot")};
        evaluation (1, 1) {mustBeA(evaluation, "Evaluation")};
      end

      data_logger.index_ = data_logger.index_ + 1;
      idx = data_logger.index_;

      data_logger.variables_log_.time(idx, 1) = time;

      LP = robot.getLinkParameter();
      SV = robot.getStateVariable();

      data_logger.variables_log_.base_position(idx, :) = SV.getBasePosition()';
      data_logger.variables_log_.base_orientation(idx, :) = SV.getBaseOrientationEuler()';
      data_logger.variables_log_.base_linear_velocity(idx, :) = SV.getBaseLinearVelocity()';
      data_logger.variables_log_.base_angular_velocity(idx, :) = SV.getBaseAngularVelocity()';
      data_logger.variables_log_.base_linear_acceleration(idx, :) = SV.getBaseLinearAcceleration()';
      data_logger.variables_log_.base_angular_acceleration(idx, :) = SV.getBaseAngularAcceleration()';

      data_logger.variables_log_.joint_angular_position(idx, :) = SV.getJointAngularPosition()';
      data_logger.variables_log_.joint_angular_velocity(idx, :) = SV.getJointAngularVelocity()';
      data_logger.variables_log_.joint_angular_acceleration(idx, :) = SV.getJointAngularAcceleration()';

      joint_torque = SV.getJointTorque();
      data_logger.variables_log_.joint_torque(idx, :) = joint_torque';
      if (data_logger.kSaveMaxJointTorque_)
        data_logger.variables_log_.max_joint_torque(idx, :) = max(abs(joint_torque));
      end
      if (data_logger.kSaveRMSJointTorque_)
        data_logger.variables_log_.rms_joint_torque(idx, :) = rms(joint_torque);
      end

      Fe = SV.getGroundReactionForce(LP);
      Te = SV.getGroundReactionMoment(LP);
      EE_positions = robot.getEEPosition();
      for limb_id = 1 : LP.getNumberOfLimb()
        data_logger.variables_log_.(matlab.lang.makeValidName("GRF_" + num2str(limb_id)))(idx, :) = Fe(:, limb_id)';
        data_logger.variables_log_.(matlab.lang.makeValidName("GRF_norm_" + num2str(limb_id)))(idx, :) = norm(Fe(:, limb_id));
        data_logger.variables_log_.(matlab.lang.makeValidName("GRM_" + num2str(limb_id)))(idx, :) = Te(:, limb_id)';
        data_logger.variables_log_.(matlab.lang.makeValidName("GRM_norm_" + num2str(limb_id)))(idx, :) = norm(Te(:, limb_id));

        data_logger.variables_log_.(matlab.lang.makeValidName("EE_position_" + num2str(limb_id)))(idx, :) = EE_positions(:, limb_id)';
      end

      if (data_logger.kSaveManipulability_)
        data_logger.variables_log_.manipulability_measure(idx, :) = evaluation.getManipulability().getManipulabilityMeasure();
      end
      if (data_logger.kSaveDynamicManipulability_)
        data_logger.variables_log_.dynamic_manipulability_measure(idx, :) = evaluation.getManipulability().getDynamicManipulabilityMeasure();
      end

      if (data_logger.kSaveTumbleStabilityMargin_)
        data_logger.variables_log_.TSM(idx, 1) = evaluation.getTumbleStabilityMargin().getTumbleStabilityMargin();
      end

      if (data_logger.kSaveGravitoInertialAcceleration_)
        data_logger.variables_log_.GIA_vector(idx, :) = evaluation.getGIA().getGIAVector()';
        data_logger.variables_log_.GIAM(idx, 1) = evaluation.getGIA().getGIAM();
        data_logger.variables_log_.GIA_inclination_margin(idx, 1) = evaluation.getGIA().getGIAInclinationMargin();
      end

      if (data_logger.kSaveCostOfTransport_)
        data_logger.variables_log_.CoT(idx, :) = evaluation.getCostOfTransport().getCoT();
      end
    end

    function saveDataFiles(data_logger, config, run_id)
    % Save variable data file and config file
      arguments (Input)
        data_logger;
        config (1, 1) {mustBeA(config, "string")};
        run_id (1, 1) {mustBeA(run_id, "string")};
      end

      if (data_logger.kSaveCsvFile_)
        data = struct2table(data_logger.variables_log_);
        writetable(data, data_logger.kSaveDataDirName_ + filesep + run_id + "_data.csv");
      end

      if (data_logger.kSaveConfigFile_)
        kConfigFileName = "config_" + config + ".m";
        kRelativePathToConfigFile = "config" + filesep + "preset" + filesep;
        copyfile(kRelativePathToConfigFile + kConfigFileName, data_logger.kSaveDataDirName_);
      end
    end

  end

  %% Getter
  methods (Access = public)

    function variables_log = getVariablesLog(data_logger)
      variables_log = data_logger.variables_log_;
    end

  end

end  % DataLogger
