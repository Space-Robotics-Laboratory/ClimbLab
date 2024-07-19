classdef SeventhOrderBezier

  properties (SetAccess = private, GetAccess = public)
    coefficients (3, 8) double;
      % 1st dim: x-y-z coordinates
      % 2nd dim: coefficient index
  end
  properties (Constant, GetAccess = private)
    % Order of the Bezier polynomial
    bezier_polynomial_order (1, 1) uint8 = 7;
  end

  methods (Access = public)
    % Constructor
    function seventh_order_bezier = SeventhOrderBezier()
      seventh_order_bezier.coefficients = zeros(3, 8);
    end

    function seventh_order_bezier = calcCoefficients(seventh_order_bezier, ...
        start_time, mid_time, final_time, ...
        start_position, mid_position, final_position, ...
        start_velocity, mid_velocity, final_velocity, ...
        start_acceleration, final_acceleration)
      arguments (Input)
        seventh_order_bezier;
        start_time         (1, 1) {mustBeA(start_time,         "double")};
        mid_time           (1, 1) {mustBeA(mid_time,           "double")};
        final_time         (1, 1) {mustBeA(final_time,         "double")};
        start_position     (3, 1) {mustBeA(start_position,     "double")};
        mid_position       (3, 1) {mustBeA(mid_position,       "double")};
        final_position     (3, 1) {mustBeA(final_position,     "double")};
        start_velocity     (3, 1) {mustBeA(start_velocity,     "double")};
        mid_velocity       (3, 1) {mustBeA(mid_velocity,       "double")};
        final_velocity     (3, 1) {mustBeA(final_velocity,     "double")};
        start_acceleration (3, 1) {mustBeA(start_acceleration, "double")};
        final_acceleration (3, 1) {mustBeA(final_acceleration, "double")};
      end
      m = double(seventh_order_bezier.bezier_polynomial_order);
      AA = zeros(3, m + 1);
      % Computing coefficients for position constraints
      AA(:, 1) = start_position;
      AA(:, m + 1) = final_position;
      % Computing coefficients for velocity constraints
      AA(:, 2) = start_velocity ./ m + AA(:, 1);
      AA(:, m) = final_velocity ./ m + AA(:, m + 1);
      % Computing coefficients for acceleration constraints
      AA(:, 3)     = start_acceleration ./ (m * (m - 1)) - AA(:, 1) + 2 * AA(:, 2);
      AA(:, m - 1) = final_acceleration ./ (m * (m - 1)) - AA(:, m + 1) + 2 * AA(:, m);

      x = zeros(3, 1);
      for i = 0:m
        x = x + nchoosek(m, i) * (mid_time / final_time).^i ...
          .* ((final_time - mid_time) / final_time).^(m - i) .* AA(1:3, i + 1);
      end
      xd = zeros(3, 1);
      for j = 0:m-1
        xd = xd + nchoosek(m - 1, j) * (mid_time / final_time).^j ...
          .* ((final_time - mid_time) / final_time).^(m - 1 - j) * m / final_time ...
          .* (AA(1:3, j + 2) - AA(1:3, j + 1));
      end

      XX = [mid_position' - x';
            mid_velocity' - xd'];

      % Coefficients
      t_d4 = m * (final_time - start_time) ...
        * (seventh_order_bezier.calcBim(start_time, final_time, mid_time, 2, m - 1) ...
        - seventh_order_bezier.calcBim(start_time, final_time, mid_time, 3, m - 1));
      t_d5 = m * (final_time - start_time) ...
        * (seventh_order_bezier.calcBim(start_time, final_time, mid_time, 3, m - 1) ...
        - seventh_order_bezier.calcBim(start_time, final_time, mid_time, 4, m - 1));

      TT = [seventh_order_bezier.calcBim(start_time, final_time, mid_time, 3, m), ...
                  seventh_order_bezier.calcBim(start_time, final_time, mid_time, 4, m);
            t_d4, t_d5];

      AA(1:3, 4:5) = (TT \ XX)';
      seventh_order_bezier.coefficients = AA;
    end

    function desired_position = calcDesiredPositionForCurrentTimeStep(seventh_order_bezier, ...
        current_time, start_time, final_time)
      arguments (Input)
        seventh_order_bezier;
        current_time (1, 1) {mustBeA(current_time, "double")};
        start_time   (1, 1) {mustBeA(start_time,   "double")};
        final_time   (1, 1) {mustBeA(final_time,   "double")};
      end
      m = double(seventh_order_bezier.bezier_polynomial_order);
      x = zeros(3, 1);
      t = current_time - start_time;
      % Compute position from the Bezier curve equation
      for k = 0:m
        x = x + nchoosek(m, k) * (t / final_time).^k .* ((final_time - t) / final_time).^(m - k) ...
          .* seventh_order_bezier.coefficients(:, k + 1);
      end
      desired_position = x;
    end

  end

  methods (Access = private)

    function Bi = calcBim(~, t0, tf, t, i, m)
      Bi = nchoosek(m, i) * (t / (tf - t0)).^i .* ((tf - t) / (tf - t0)).^(m - i);
    end

  end

end
% EOF