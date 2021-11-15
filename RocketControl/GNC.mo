within RocketControl;

package GNC
  extends Internal.Icons.Navigation;
  model PitchController
    Modelica.Blocks.Interfaces.RealOutput finDeflection[4] annotation(
      Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput u annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const(k = 0) annotation(
      Placement(visible = true, transformation(origin = {-84, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain(k = -1) annotation(
      Placement(visible = true, transformation(origin = {-30, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(u, gain.u) annotation(
      Line(points = {{-100, 0}, {-62, 0}, {-62, -70}, {-42, -70}}, color = {0, 0, 127}));
    connect(const.y, finDeflection[1]) annotation(
      Line(points = {{-72, 84}, {42, 84}, {42, 0}, {106, 0}}, color = {0, 0, 127}));
    connect(u, finDeflection[2]) annotation(
      Line(points = {{-100, 0}, {106, 0}}, color = {0, 0, 127}));
    connect(const.y, finDeflection[3]) annotation(
      Line(points = {{-72, 84}, {42, 84}, {42, 0}, {106, 0}}, color = {0, 0, 127}));
    connect(gain.y, finDeflection[4]) annotation(
      Line(points = {{-18, -70}, {42, -70}, {42, 0}, {106, 0}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Text(extent = {{-100, 80}, {100, -80}}, textString = "Pitch")}));
  end PitchController;

  package Navigation
    model DiscreteKalmanFilter
      parameter Integer n(min = 1) "Number of states" annotation(
        Evaluate = true);
      parameter Integer m(min = 0) "Number of inputs" annotation(
        Evaluate = true);
      parameter Integer p(min = 1) "Number of outputs" annotation(
        Evaluate = true);
      parameter Real A[n, n];
      parameter Real B[n, max(m, 1)] = zeros(n, max(m, 1)) annotation(
        Dialog(enable = m > 0));
      parameter Real C[p, n];
      parameter Real P0[n, n] = identity(n) "Initial state covariance matrix";
      parameter Real Q[n, n] "Process covariance matrix";
      parameter Real R[p, p] "Output covariance matrix";
      parameter Real x0[n] "Initial state";
      Real P[n, n](start = P0) "P(k|k)";
      Real Pest[n, n](start = P0) "P(k|k-1)";
      Real L[n, p];
      Real x_pred[n](start = x0);
      Modelica.Blocks.Interfaces.RealInput y_meas[p] annotation(
        Placement(visible = true, transformation(origin = {-106, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput x_est[n](start = x0, each fixed = true) annotation(
        Placement(visible = true, transformation(origin = {106, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput y_est[p](start = C * x0, each fixed = true) annotation(
        Placement(visible = true, transformation(origin = {106, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput u[max(m, 1)] annotation(
        Placement(visible = true, transformation(origin = {-106, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.BooleanInput enable annotation(
        Placement(visible = true, transformation(origin = {0, 108}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 102}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    equation
      when Clock() then
        if m == 0 then
          u[1] = sample(0);
        end if;
        if not sample(enable) then
          Pest = P0;
          L = zeros(n, p);
          P = P0;
          x_pred = x0;
          x_est = x0;
          y_est = C * x_est;
        else
          Pest = A * previous(P) * transpose(A) + Q;
          L = transpose(Modelica.Math.Matrices.solve2(transpose(C * Pest * transpose(C) + R), transpose(Pest * transpose(C))));
          P = Pest - L * C * Pest;
          x_pred = A * previous(x_est) + B * u;
          x_est = x_pred + L * (y_meas - C * x_pred);
          y_est = C * x_est;
        end if;
      end when;
      annotation(
        Icon(graphics = {Text(extent = {{-100, 100}, {100, -100}}, textString = "KF"), Text(origin = {102, 58.24}, lineColor = {128, 128, 128}, extent = {{-246, 56.76}, {-164, 23.76}}, textString = "y_meas"), Text(origin = {-22, -53}, lineColor = {128, 128, 128}, extent = {{-108, 43}, {-72, 18}}, textString = "u"), Text(origin = {300, 52.24}, lineColor = {128, 128, 128}, extent = {{-246, 56.76}, {-164, 23.76}}, textString = "x_est"), Text(origin = {304, -69.76}, lineColor = {128, 128, 128}, extent = {{-246, 56.76}, {-164, 23.76}}, textString = "y_est")}));
    end DiscreteKalmanFilter;

    model AttitudeEstimation "Rocket attitude estimation from gyroscope and magnetometer data, using a Multiplicative Extended Kalamn Filter (MEKF)"
      import Modelica.Mechanics.MultiBody.Frames.Quaternions;
      extends RocketControl.GNC.Internal.Icons.Navigation;
      parameter Integer samplingPeriodMs;
      parameter NonSI.Angle_deg heading0;
      parameter NonSI.Angle_deg elevation0;
      parameter NonSI.Angle_deg roll0 = 0;
      parameter SI.AngularVelocity bias0[3] = {0, 0, 0};
      parameter Real P0[6, 6] = identity(6);
      parameter Real sigma_v "Attitude noise";
      parameter SI.AngularVelocity sigma_u "Bias noise";
      parameter Real sigma_b "Normalized magnetomer measurement noise";
      SI.AngularVelocity bias[3](start = zeros(3));
      Modelica.Blocks.Interfaces.RealInput w_meas_degs[3] annotation(
        Placement(visible = true, transformation(origin = {-100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput b_meas_nt[3] annotation(
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput r_0_est[3] annotation(
        Placement(visible = true, transformation(origin = {-100, -70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput q_est[4] annotation(
        Placement(visible = true, transformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput w_est_degs[3](each final quantity = "AngularVelocity", each final unit = "deg/s") annotation(
        Placement(visible = true, transformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    protected
      final parameter SI.Duration T = samplingPeriodMs / 1000;
      final parameter Modelica.Mechanics.MultiBody.Frames.Orientation R0 = Modelica.Mechanics.MultiBody.Frames.axesRotations({3, 2, 1}, from_deg({heading0, elevation0, roll0}), {0, 0, 0});
      final parameter Real q0[4] = Quaternions.from_T(R0.T);
      final parameter Real x0[6] = {0, 0, 0, 0, 0, 0};
      final parameter Real R[3, 3] = identity(3) * sigma_b ^ 2;
      Real q_prop[4](start = q0);
      Real q_prev[4];
      Real q_update[4];
      Real q_star[4];
      SI.MagneticFluxDensity b_n[3];
      Real b_n_vers[3];
      Real b_b_vers[3];
      Real P_prop[6, 6](start = P0);
      Real P_update[6, 6];
      Real K[6, 3];
      Real H[3, 6];
      Real h[3];
      Real Aq[3, 3];
      Real x_update[6];
      Real F[6, 6];
      Real Q[6, 6];
      SI.AngularVelocity w_meas[3];
      SI.MagneticFluxDensity b_meas[3];
      SI.AngularVelocity w_est[3];

      function Fmatrix
        input Real[3] w;
        output Real[6, 6] F;
      protected
        Real wnorm;
        Real Ftemp[6, 6];
      algorithm
//    Ftemp := [-skew(w), -identity(3); zeros(3, 3), zeros(3, 3)];
//    F := identity(6) + T * Ftemp;
        wnorm := norm(w);
        F := zeros(6, 6);
        F[1:3, 1:3] := identity(3) - skew(w) * sin(wnorm * T) / wnorm + skew(w) ^ 2 * (1 - cos(wnorm * T)) / wnorm ^ 2;
        F[1:3, 4:6] := skew(w) * (1 - cos(wnorm * T)) ./ wnorm ^ 2 - identity(3) * T - skew(w) ^ 2 * (wnorm * T - sin(wnorm * T)) ./ wnorm ^ 3;
        F[4:6, 4:6] := identity(3);
      end Fmatrix;

      function Qmatrix
        input Real[3] w;
        input Real sigma_v;
        input Real sigma_u;
        output Real Q[6, 6];
      protected
        Real wnorm;
        Real Qtemp[6, 6];
        constant Real G[6, 6] = [-identity(3), zeros(3, 3); zeros(3, 3), identity(3)];
      algorithm
        wnorm := norm(w);
        if wnorm > 1e-5 then
          Q[1:3, 1:3] := sigma_v ^ 2 * T * identity(3) + sigma_u ^ 2 * (1 / 3 * T ^ 3 * identity(3) - skew(w) ^ 2 * (2 * wnorm * T - 2 * sin(wnorm * T) - 1 / 3 * wnorm ^ 3 * T ^ 3) / wnorm ^ 5);
          Q[1:3, 4:6] := sigma_u ^ 2 * (skew(w) * (wnorm * T - sin(wnorm * T)) / wnorm ^ 3 - 0.5 * T ^ 2 * identity(3) - skew(w) ^ 2 * (0.5 * wnorm ^ 2 * T ^ 2 + cos(wnorm * T) - 1) / wnorm ^ 4);
          Q[4:6, 1:3] := transpose(Q[1:3, 4:6]);
          Q[4:6, 4:6] := sigma_u ^ 2 * T * identity(3);
        else
          Qtemp := [identity(3) * sigma_v ^ 2, zeros(3, 3); zeros(3, 3), identity(3) * sigma_u ^ 2];
          Q := T * G * Qtemp * transpose(G);
        end if;
      end Qmatrix;

      function quatSkew
        input Real q[4];
        output Real E[4, 3];
      algorithm
        E[1, 1] := q[4];
        E[1, 2] := -q[3];
        E[1, 3] := q[2];
        E[2, 1] := q[3];
        E[2, 2] := q[4];
        E[2, 3] := -q[1];
        E[3, 1] := -q[2];
        E[3, 2] := q[1];
        E[3, 3] := q[4];
        E[4, 1] := -q[1];
        E[4, 2] := -q[2];
        E[4, 3] := -q[3];
      end quatSkew;

      function propagateQuaternion
        input Real[4] q;
        input Real[3] w;
        output Real[4] q_prop;
      protected
        Real[4, 4] O;
        Real[3] psi;
        Real wnorm;
      algorithm
        wnorm := norm(w);
        if wnorm > 1e-7 then
          psi := sin(0.5 * wnorm * T) * w / wnorm;
          O := [cos(0.5 * wnorm * T) * identity(3) - skew(psi), matrix(psi); -transpose(matrix(psi)), cos(0.5 * wnorm * T)];
        else
          O := identity(4);
        end if;
        q_prop := O * q;
      end propagateQuaternion;

      outer RocketControl.World.MyWorld world;
    equation
      when Clock() then
// Unit conversion
        w_meas = from_deg(w_meas_degs);
        b_meas = b_meas_nt * 1e-9;
        q_prev = previous(q_prop);
        q_est = q_prev;
        Aq = Quaternions.to_T(q_prev);
// Update
        b_n = world.magneticField(r_0_est);
        b_n_vers = b_n / norm(b_n);
        b_b_vers = b_meas / norm(b_meas);
        h = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve2(q_prev, b_n_vers);
        H = [skew(h), zeros(3, 3)];
        K = transpose(Modelica.Math.Matrices.solve2(transpose(H * previous(P_prop) * transpose(H) + R), transpose(previous(P_prop) * transpose(H))));
        P_update = (identity(6) - K * H) * previous(P_prop);
        x_update = K * (b_b_vers - h);
// Reset
        q_star = q_prev + 0.5 * quatSkew(q_prev) * x_update[1:3];
        q_update = q_star / norm(q_star);
        bias = previous(bias) + x_update[4:6];
// Propagate
        w_est = w_meas - bias;
        w_est_degs = to_deg(w_est);
        F = Fmatrix(w_est);
        Q = Qmatrix(w_est, sigma_v, sigma_u);
        P_prop = F * P_update * transpose(F) + Q;
        q_prop = propagateQuaternion(q_update, w_est);
      end when;
      annotation(
        Icon(graphics = {Text(origin = {134, 84}, lineColor = {111, 111, 111}, extent = {{-30, 20}, {30, -20}}, textString = "q"), Text(origin = {134, -16}, lineColor = {111, 111, 111}, extent = {{-30, 20}, {30, -20}}, textString = "w"), Text(origin = {-152, 120}, lineColor = {111, 111, 111}, extent = {{-40, 20}, {40, -20}}, textString = "w"), Text(origin = {-148, 40}, lineColor = {111, 111, 111}, extent = {{-40, 20}, {40, -20}}, textString = "b"), Text(origin = {-146, -40}, lineColor = {111, 111, 111}, extent = {{-40, 20}, {40, -20}}, textString = "r_0"), Text(origin = {0, -76}, fillColor = {102, 102, 102}, extent = {{-100, 18}, {100, -18}}, textString = "asset"), Line(origin = {-97, 0}, points = {{-3, 80}, {3, 80}, {3, -80}, {-3, -80}}), Line(origin = {-95, 0}, points = {{-5, 0}, {5, 0}}), Line(origin = {95, 25}, points = {{-5, -25}, {-1, -25}, {-1, 25}, {5, 25}}), Line(origin = {102, -25}, points = {{-8, 25}, {-8, -25}, {8, -25}})}));
    end AttitudeEstimation;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Navigation;

  package Internal
  package Icons
      model Navigation
  equation

        annotation(
          Icon(graphics = {Rectangle(fillColor = {252, 255, 252}, fillPattern = FillPattern.Solid, extent = {{-90, 90}, {90, -90}}, radius = 50), Text(origin = {2, -262}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name"), Line(origin = {52.2533, 55.5403}, points = {{-18, -18}, {18, 18}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Line(origin = {25.5114, -29.1575}, rotation = -90, points = {{-18, -18}, {24, 24}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Text(origin = {2, -262}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name"), Line(origin = {-43.1155, -11.4371}, rotation = 180, points = {{-18, -18}, {36, -18}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Polygon(origin = {-46, -40}, fillColor = {91, 91, 91}, fillPattern = FillPattern.Solid, points = {{-2, -12}, {8, -22}, {22, 12}, {12, 22}, {-22, 8}, {-2, -12}}), Polygon(origin = {-15, -9}, fillColor = {155, 183, 193}, fillPattern = FillPattern.Solid, points = {{-43, -33}, {25, 35}, {45, 45}, {35, 25}, {-33, -43}, {-37, -39}, {-43, -33}})}));
      end Navigation;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end Icons;
  equation

    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Internal;
end GNC;
