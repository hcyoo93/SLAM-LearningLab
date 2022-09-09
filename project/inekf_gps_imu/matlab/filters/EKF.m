classdef EKF < handle

  properties
    mu; % pose = [x;v;theta] - 9*1
    Sigma;
    Q_w_mat; % gyro noise
    Q_a_mat; % acc noise
    %mu_sym;
    %F_sym;
    %W_sym;
    V; %observation noise of position
    mu_lam;
    F_lam;
    W_lam;
    H_mat;
  end

  methods

    function obj = EKF(theta0, p0, v0, V_in)

      if nargin == 0
        theta0 = zeros(3, 1);
        p0 = zeros(3, 1);
        v0 = zeros(3, 1);
        V_in = eye(3) * 0.01;
      end

      % TODO: Complete following varialbe computation
      obj.mu;
      obj.Sigma;

      obj.Q_w_mat;
      obj.Q_a_mat;
      obj.V;

      obj.mu_lam = @(x1, x2, x3, v1, v2, v3, t1, t2, t3, w1, w2, w3, a1, a2, a3, dt) [];

      obj.F_lam = @(x1, x2, x3, v1, v2, v3, t1, t2, t3, w1, w2, w3, a1, a2, a3, dt) [];

      obj.W_lam = @(x1, x2, x3, v1, v2, v3, t1, t2, t3, w1, w2, w3, a1, a2, a3, dt) [];

      obj.H_mat;

    end

    %------------------------------------------------------------------

    function prediction(obj, w, a, dt)
      F_mat = obj.eval_F(obj.mu, w, a, dt);
      W_mat = obj.eval_W(obj.mu, w, a, dt);

      % TODO: Complete following varialbe computation
      obj.Sigma;
      % Propoagate mean through non-linear dynamics
      obj.mu;
    end

    %------------------------------------------------------------------

    function correction(obj, gps)
      nu = gps - obj.mu(1:3);
      H = obj.H_mat;

      S = H * obj.Sigma * H' + obj.V;
      K = obj.Sigma * H' / S;

      % TODO: Complete following varialbe computation
      obj.mu;
      obj.Sigma;
    end

    %------------------------------------------------------------------

    function rotm = rot_mat(~, euler_angle)
      r = euler_angle(1);
      p = euler_angle(2);
      y = euler_angle(3);

      % http://planning.cs.uiuc.edu/node102.html
      rx = [1, 0, 0; 0, cos(r), -sin(r); 0, sin(r), cos(r)];
      ry = [cos(p), 0, sin(p); 0, 1, 0; -sin(p), 0, cos(p)];
      rz = [cos(y), -sin(y), 0; sin(y), cos(y), 0; 0, 0, 1];
      rotm = rz * ry * rx;
    end

    %------------------------------------------------------------------

    function out = eval_mu(obj, mu, w, a, dt)
      x1 = mu(1); x2 = mu(2); x3 = mu(3);
      v1 = mu(4); v2 = mu(5); v3 = mu(6);
      t1 = mu(7); t2 = mu(8); t3 = mu(9);
      w1 = w(1); w2 = w(2); w3 = w(3);
      a1 = a(1); a2 = a(2); a3 = a(3);

      out = obj.mu_lam(x1, x2, x3, v1, v2, v3, t1, t2, t3, w1, w2, w3, a1, a2, a3, dt);
      %double(subs(obj.mu_sym));
    end

    function out = eval_F(obj, mu, w, a, dt)
      x1 = mu(1); x2 = mu(2); x3 = mu(3);
      v1 = mu(4); v2 = mu(5); v3 = mu(6);
      t1 = mu(7); t2 = mu(8); t3 = mu(9);
      w1 = w(1); w2 = w(2); w3 = w(3);
      a1 = a(1); a2 = a(2); a3 = a(3);

      out = obj.F_lam(x1, x2, x3, v1, v2, v3, t1, t2, t3, w1, w2, w3, a1, a2, a3, dt);
      %double(subs(obj.F_sym));
    end

    function out = eval_W(obj, mu, w, a, dt)
      x1 = mu(1); x2 = mu(2); x3 = mu(3);
      v1 = mu(4); v2 = mu(5); v3 = mu(6);
      t1 = mu(7); t2 = mu(8); t3 = mu(9);
      w1 = w(1); w2 = w(2); w3 = w(3);
      a1 = a(1); a2 = a(2); a3 = a(3);

      out = obj.W_lam(x1, x2, x3, v1, v2, v3, t1, t2, t3, w1, w2, w3, a1, a2, a3, dt);
      %double(subs(obj.W_sym));
    end

  end

end
