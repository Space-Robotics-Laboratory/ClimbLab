classdef GIAStabilityPolyhedron < handle
% Gravito-Inertial Acceleration Stability Polyhedron
%
% Created     : 2020.01.20 by Warley Ribeiro
% Last updated: 2024.12.25 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
  end
  properties (SetAccess = private, GetAccess = public)
    plane_point_;  % Maximum accelerations in a normal direction of tumbling axis [m/s^2] (3 x n matrix)
    plane_vector_;  % Normal vector to tumbling axis from center of gravity (3 x n matrix)
  end

  %% Public Methods
  methods (Access = public)

    function GIA_stability_polyhedron = GIAStabilityPolyhedron(config_evaluation)
    % Constructor
      arguments (Input)
        config_evaluation (1, 1) {mustBeA(config_evaluation, "ConfigEvaluation")};
      end

    end

  end

  %% Setter
  methods (Access = public)

    function setPlanePoint(GIA_stability_polyhedron, max_acceleration_in_normal_direction)
      GIA_stability_polyhedron.plane_point_ = max_acceleration_in_normal_direction;
    end

    function setPlaneVector(GIA_stability_polyhedron, normal_vector_to_tumbling_axis_from_CoG)
      GIA_stability_polyhedron.plane_vector_ = normal_vector_to_tumbling_axis_from_CoG;
    end

  end

  %% Getter
  methods (Access = public)

    function plane_point = getPlanePoint(GIA_stability_polyhedron)
      plane_point = GIA_stability_polyhedron.plane_point_;
    end

    function plane_vector = getPlaneVector(GIA_stability_polyhedron)
      plane_vector = GIA_stability_polyhedron.plane_vector_;
    end

  end

end  % GIAStabilityPolyhedron
