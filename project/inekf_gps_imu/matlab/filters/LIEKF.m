classdef LIEKF < handle
  %% Left-Invariant filter class, predicts next state, corrects prediction
  properties
    mu; %Pose Mean
    bias; %Bias of gyro and accelerometer = [wb,ab]';
    Sigma; %Pose Sigma
    A; %Process model - Not currently being used
    cov_g; %gyro noise
    cov_a; %acc noise
    cov_gb; %gyro bias
    cov_ab; %acc bias
    V; %observation noise of position
    Q; %all covariance in process model
    sigma_cart
  end

  methods

    function obj = LIEKF(R0, p0, v0, cov_g_, cov_a_, cov_gb_, cov_ab_, V_)
      % Set the initial state
      if nargin == 0
        R0 = eye(3);
        p0 = zeros(3, 1);
        v0 = zeros(3, 1);
        cov_g_ = eye(3) * 20;
        cov_a_ = eye(3) * 20;
        cov_gb_ = eye(3);
        cov_ab_ = eye(3);
        V_ = [4.6778 1.9437 0.0858;
            1.9437 11.5621 5.8445;
            0.0858 5.8445 22.4051] * 1000;
      end

      % TODO: Complete following varialbe computation
      obj.mu;

      obj.Sigma; %TBT
      obj.bias;

      obj.cov_g = cov_g_; %TBT
      obj.cov_a = cov_a_;
      obj.cov_gb = cov_gb_;
      obj.cov_ab = cov_ab_;
      obj.V = V_;

      obj.Q;
      obj.A;
    end

    function [R, p, v] = getState(obj)
      R = obj.mu(1:3, 1:3);
      v = obj.mu(1:3, 4);
      p = obj.mu(1:3, 5);
    end

    function prediction(obj, w, a, dt) %TBC bias
      skew = @(u) obj.skew(u);

      % TODO: Complete following varialbe computation
      % Predicts position from gyro/accelerometer data
      if norm(w) > 1e-8
        gamma0;

        gamma1;

        gamma2;
      else
        gamma0;
        gamma1;
        gamma2;
      end

      % Store state in convenient variables
      [R, p, v] = obj.getState();

      % Caclulate predicted state and covariance
      obj.mu;
      obj.Sigma;
    end

    % GPS 3x1 is this in R^3 ECEF/NED/ENU??
    function correction(obj, GPS)
      % TODO: Complete following varialbe computation
      obj.mu;
      obj.bias;
      obj.Sigma;
    end

    function skew = skew(obj, u)
      % TODO: Complete following varialbe computation
    end

    function xi = makeTwist(obj, delta)
      % TODO: Complete following varialbe computation

    end

  end

end
