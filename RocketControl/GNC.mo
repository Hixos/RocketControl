within RocketControl;

package GNC
  extends Internal.Icons.Navigation;

  block DiscreteKalmanFilter
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
    Modelica.Blocks.Interfaces.BooleanInput correct annotation(
      Placement(visible = true, transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
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
        x_pred = A * previous(x_est) + B * u;
        if correct then
          L = transpose(Modelica.Math.Matrices.solve2(transpose(C * Pest * transpose(C) + R), transpose(Pest * transpose(C))));
          P = Pest - L * C * Pest;
          x_est = x_pred + L * (y_meas - C * x_pred);
        else
          L = zeros(n, p);
          P = Pest;
          x_est = x_pred;
        end if;
        y_est = C * x_est;
      end if;
    end when;
    annotation(
      Icon(graphics = {Text(extent = {{-100, 100}, {100, -100}}, textString = "KF"), Text(origin = {102, 58.24}, lineColor = {128, 128, 128}, extent = {{-246, 56.76}, {-164, 23.76}}, textString = "y_meas"), Text(origin = {-22, -53}, lineColor = {128, 128, 128}, extent = {{-108, 43}, {-72, 18}}, textString = "u"), Text(origin = {300, 52.24}, lineColor = {128, 128, 128}, extent = {{-246, 56.76}, {-164, 23.76}}, textString = "x_est"), Text(origin = {304, -69.76}, lineColor = {128, 128, 128}, extent = {{-246, 56.76}, {-164, 23.76}}, textString = "y_est"), Text(origin = {246, -135.76}, lineColor = {128, 128, 128}, extent = {{-246, 56.76}, {-164, 23.76}}, textString = "corr")}));
  end DiscreteKalmanFilter;

  block ContinuousLQR
  extends RocketControl.Interfaces.PartialConditionalEnablePort;
    parameter Integer n(min = 1) = 1 annotation(
      Evaluate = true);
    parameter Integer m(min = 1) = 1 annotation(
      Evaluate = true);
    parameter Integer maxiter(min = 1) = 50;
    parameter Real tol(min = 0) = 1e-13;
    parameter Real g = -1;
    Modelica.Blocks.Interfaces.RealInput A[n, n] annotation(
      Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput B[n, m] annotation(
      Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput Q[n, n] annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput R[m, m] annotation(
      Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput u[m](start = zeros(m)) annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput x[n] annotation(
      Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Real P[n, n];
    Integer niter;
    Modelica.Blocks.Interfaces.RealOutput K[m, n] annotation(
      Placement(visible = true, transformation(origin = {110, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    if enable then
      (P, niter) = RocketControl.Math.sdaCare(A, B, R, Q, g, tol, maxiter);
      K = Modelica.Math.Matrices.solve2(R, transpose(B) * P);
      u = -K * x;
//  assert(niter < maxiter, "sdaCare maximum number of iterations reached", level = AssertionLevel.warning);
//      when niter == maxiter then
//       terminate("Too many iterations");
//      end when;
    else
      P = zeros(n, n);
      K = zeros(m, n);
      niter = 0;
      u = zeros(m);
    end if;
    annotation(
      Icon(graphics = {Polygon(fillColor = {247, 219, 255}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-100, 100}, {-100, -80}, {-100, -100}, {70, -100}, {100, -80}, {100, 80}, {70, 100}, {-80, 100}}), Text(origin = {-2, 1}, extent = {{-60, 59}, {60, -59}}, textString = "LQ"), Text(origin = {-80, 80}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "A"), Text(origin = {-80, 40}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "B"), Text(origin = {-80, 0}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "Q"), Text(origin = {-80, -40}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "R"), Text(origin = {80, 0}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "u"), Rectangle(origin = {-100, -80}, fillColor = {217, 255, 215}, fillPattern = FillPattern.Solid, extent = {{-40, 20}, {40, -20}}), Text(origin = {-80, -80}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "x"), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {80, 52}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "K")}));
  end ContinuousLQR;

  block AugmentIntegratorState
  parameter Integer n(min = 1) = 1 annotation(
      Evaluate = true);
    parameter Integer m(min = 1) = 1 annotation(
      Evaluate = true);
    parameter Integer p(min = 1) = 1 annotation(
      Evaluate = true);
    parameter Boolean use_D = false;
    Modelica.Blocks.Interfaces.RealInput B[n, m] annotation(
      Placement(visible = true, transformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput A[n, n] annotation(
      Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput C[p, n] annotation(
      Placement(visible = true, transformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput Aaug[n + p, n + p] annotation(
      Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput Baug[n + p, m] annotation(
      Placement(visible = true, transformation(origin = {110, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput Caug[p, n + p] annotation(
      Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput D[p, m] annotation(
      Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  equation
    if not use_D then
      D = zeros(p, m);
    end if;
    Aaug = [A, zeros(n, p); -C, zeros(p, p)];
    Baug = [B; -D];
    Caug = [C, zeros(p, p)];
    annotation(
      Icon(graphics = {Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Rectangle(lineColor = {255, 85, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {-79, 81}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "A"), Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "der(e)"), Text(origin = {-80, -28}, lineColor = {102, 102, 102}, extent = {{-20, 18}, {20, -18}}, textString = "C"), Text(origin = {-79, 31}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "B"), Text(origin = {77, 49}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "Aaug"), Text(origin = {77, -45}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "Baug"), Text(origin = {-80, -78}, lineColor = {102, 102, 102}, extent = {{-20, 18}, {20, -18}}, textString = "D")}));
  end AugmentIntegratorState;

  model FeedbackIntegrator
  extends RocketControl.Components.Interfaces.PartialConditionalEnablePort;
    parameter Integer n(min = 1) = 1;
    Modelica.Blocks.Interfaces.RealInput ref[n] annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput err_int[n] annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput feedback[n] annotation(
      Placement(visible = true, transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  equation
    if enable then
//err_int = zeros(n);
      der(err_int) = ref - feedback;
    else
      err_int = zeros(n);
    end if;
    annotation(
      Icon(graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Line(visible = false, points = {{50, 70}, {80, 70}}, color = {255, 0, 0}), Line(visible = false, points = {{-80, -70}, {-50, -70}}, color = {255, 0, 0}), Ellipse(origin = {-50, 0}, extent = {{-24, 24}, {24, -24}}), Text(origin = {40, -14}, extent = {{-26, 26}, {26, -26}}, textString = "s"), Rectangle(origin = {40, 0}, extent = {{-40, 40}, {40, -40}}), Text(origin = {40, 20}, extent = {{-26, 26}, {26, -26}}, textString = "1"), Line(origin = {39.72, -2.6}, points = {{-19.8536, 2.20711}, {20.1464, 2.20711}}), Line(origin = {-35.75, -18.42}, points = {{-19.8536, 2.20711}, {-5.8536, 2.20711}}), Line(origin = {-49.88, -2.21}, points = {{-19.8536, 2.20711}, {-5.8536, 2.20711}}), Line(origin = {-43.18, 4.73}, points = {{-19.8536, 2.20711}, {-19.8536, -11.7929}}), Line(origin = {-87, 0}, points = {{-13, 0}, {13, 0}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 6), Line(origin = {-25, -62}, points = {{25, -38}, {25, -18}, {-25, -18}, {-25, 38}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 6), Line(origin = {-12.5967, -0.168941}, points = {{-13, 0}, {13, 0}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 6), Line(origin = {90, 0}, points = {{-10, 0}, {10, 0}})}));
  end FeedbackIntegrator;

  package Guidance
    model AngularRateVelocityTrack
      extends Icon;
      parameter Real k;
      Modelica.Blocks.Interfaces.RealOutput w_ref[3](each final unit = "rad/s", each final quantity = "AngularVelocity", each displayUnit = "deg/s") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput v_ref[3](each final unit = "m/s", each final quantity = "Velocity") annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      RocketControl.Components.Interfaces.AvionicsBus bus annotation(
        Placement(visible = true, transformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SI.Angle angle_err;
      Real rotation_vector[3];
      Real rotation_versor[3];
      SI.AngularVelocity[3] w_body;
    equation
      if norm(bus.v_est) > 1 and norm(v_ref) > 1 then
        angle_err = acos(bus.v_est * v_ref / (norm(bus.v_est) * norm(v_ref)));
        rotation_vector = cross(bus.v_est, v_ref);
        if norm(rotation_vector) > 1e-6 then
          rotation_versor = rotation_vector / norm(rotation_vector);
        else
          rotation_versor = {0, 0, 0};
// Singularity: no rotation. TODO: Handle case where velocities are opposing
        end if;
      else
        angle_err = 0;
        rotation_vector = {0, 0, 0};
        rotation_versor = {0, 0, 0};
      end if;
      w_body = k * Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve2(bus.q_est, rotation_versor) * angle_err;
      w_ref = cat(1, {0}, w_body[2:3]);
      annotation(
        Icon(graphics = {Text(origin = {1, 6}, extent = {{-79, 52}, {79, -52}}, textString = "w_ref")}));
    end AngularRateVelocityTrack;

    model Icon
    equation

      annotation(
        Icon(coordinateSystem(grid = {2, 0}), graphics = {Text(origin = {0, -10}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Rectangle(fillColor = {207, 223, 231}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 30)}));
    end Icon;

    model VelocityRef
      extends Icon;
      parameter SI.Angle track_ref(displayUnit = "deg");
      parameter SI.Angle climbangle_max(displayUnit = "deg") = from_deg(84);
      Modelica.Blocks.Interfaces.RealOutput v_ref[3](each final unit = "m/s", each final quantity = "Velocity") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.Interfaces.AvionicsBus bus annotation(
        Placement(visible = true, transformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SI.Velocity v_horiz;
      SI.Velocity v_horiz_min;
      SI.Velocity v_horiz_ref[3];
    protected
      final parameter Real z[3] = {0, 0, 1};
      final parameter Real dir_horiz[3] = {cos(track_ref), sin(track_ref), 0};
    equation
      v_horiz = norm(bus.v_est - bus.v_est * z * z);
      v_horiz_min = norm(bus.v_est) * cos(climbangle_max);
      v_horiz_ref = max(v_horiz, v_horiz_min) * dir_horiz;
      if norm(bus.v_est) > 1 then
        v_ref = v_horiz_ref + sqrt(norm(bus.v_est) ^ 2 - norm(v_horiz_ref) ^ 2) * z * sign(bus.v_est * z);
      else
        v_ref = bus.v_est;
      end if;
      annotation(
        Icon(graphics = {Text(origin = {-4, 11}, extent = {{-82, 55}, {82, -55}}, textString = "v_ref")}));
    end VelocityRef;

    model AngularRateGuidance
      Components.Interfaces.AvionicsBus bus annotation(
        Placement(visible = true, transformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput w_ref(displayUnit = "deg/s", quantity = "AngularVelocity", unit = "rad/s") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation

      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end AngularRateGuidance;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Guidance;

  package Navigation
extends Internal.Icons.Navigation;
    model AttitudeEstimation "Rocket attitude estimation from gyroscope and magnetometer data, using a Multiplicative Extended Kalamn Filter (MEKF)"
      import Modelica.Mechanics.MultiBody.Frames.Quaternions;
      extends RocketControl.GNC.Internal.Icons.Navigation;
      parameter Integer samplingPeriodMs;
      parameter SI.Angle heading0;
      parameter SI.Angle elevation0;
      parameter SI.Angle roll0 = 0;
      parameter Types.AngularVelocity bias0[3] = {0, 0, 0};
      parameter Real P0[6, 6] = identity(6);
      parameter Real sigma_v "Attitude noise";
      parameter Types.AngularVelocity sigma_u "Bias noise";
      parameter Real sigma_b "Normalized magnetomer measurement noise";
      Types.AngularVelocity bias[3](start = zeros(3));
      Modelica.Blocks.Interfaces.RealInput w_meas[3](each quantity = "AngularVelocity", each unit = "rad/s", each displayUnit = "deg/s") annotation(
        Placement(visible = true, transformation(origin = {-100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput b_meas[3](each final quantity = "MagneticFluxDensity", each final unit = "T", each displayUnit = "nT") annotation(
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput r_0_est[3](each quantity = "Position", each unit = "m") annotation(
        Placement(visible = true, transformation(origin = {-100, -70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput q_est[4] annotation(
        Placement(visible = true, transformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput w_est[3](each final quantity = "AngularVelocity", each final unit = "rad/s", each displayUnit = "deg/s") annotation(
        Placement(visible = true, transformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      //protected
      final parameter SI.Duration T = samplingPeriodMs / 1000;
      final parameter Modelica.Mechanics.MultiBody.Frames.Orientation R0 = Modelica.Mechanics.MultiBody.Frames.axesRotations({3, 2, 1}, {heading0, elevation0, roll0}, {0, 0, 0});
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

      function Fmatrix
        input Real[3] w;
        output Real[6, 6] F;
      protected
        Real wnorm;
        Real Ftemp[6, 6];
      algorithm
        wnorm := norm(w);
        if abs(wnorm) > 1e-5 then
          F := zeros(6, 6);
          F[1:3, 1:3] := identity(3) - skew(w) * sin(wnorm * T) / wnorm + skew(w) ^ 2 * (1 - cos(wnorm * T)) / wnorm ^ 2;
          F[1:3, 4:6] := skew(w) * (1 - cos(wnorm * T)) ./ wnorm ^ 2 - identity(3) * T - skew(w) ^ 2 * (wnorm * T - sin(wnorm * T)) ./ wnorm ^ 3;
          F[4:6, 4:6] := identity(3);
        else
          Ftemp := [-skew(w), -identity(3); zeros(3, 3), zeros(3, 3)];
          F := identity(6) + T * Ftemp;
        end if;
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
        F = Fmatrix(w_est);
        Q = Qmatrix(w_est, sigma_v, sigma_u);
        P_prop = F * P_update * transpose(F) + Q;
        q_prop = propagateQuaternion(q_update, w_est);
      end when;
      annotation(
        Icon(graphics = {Text(origin = {134, 84}, lineColor = {111, 111, 111}, extent = {{-30, 20}, {30, -20}}, textString = "q"), Text(origin = {134, -16}, lineColor = {111, 111, 111}, extent = {{-30, 20}, {30, -20}}, textString = "w"), Text(origin = {-152, 120}, lineColor = {111, 111, 111}, extent = {{-40, 20}, {40, -20}}, textString = "w"), Text(origin = {-148, 40}, lineColor = {111, 111, 111}, extent = {{-40, 20}, {40, -20}}, textString = "b"), Text(origin = {-146, -40}, lineColor = {111, 111, 111}, extent = {{-40, 20}, {40, -20}}, textString = "r_0"), Text(origin = {0, -76}, fillColor = {102, 102, 102}, extent = {{-100, 18}, {100, -18}}, textString = "asset"), Line(origin = {-97, 0}, points = {{-3, 80}, {3, 80}, {3, -80}, {-3, -80}}), Line(origin = {-95, 0}, points = {{-5, 0}, {5, 0}}), Line(origin = {95, 25}, points = {{-5, -25}, {-1, -25}, {-1, 25}, {5, 25}}), Line(origin = {102, -25}, points = {{-8, 25}, {-8, -25}, {8, -25}})}));
    end AttitudeEstimation;

    model PositionEstimation
      extends RocketControl.GNC.Internal.Icons.Navigation;
      parameter SI.Position sigma_pos;
      parameter SI.Position sigma_vel;
      parameter Real[6] sigma_gps;
      parameter Real[6] x0 = {0, 0, 0, 0, 0, 0};
      parameter Integer samplingPeriodMs;
      final parameter SI.Duration T = samplingPeriodMs / 1000;
      RocketControl.GNC.DiscreteKalmanFilter discreteKalmanFilter(A = [identity(3), identity(3) * T; zeros(3, 3), identity(3)], B = [zeros(3, 3); identity(3) * T], C = identity(6), Q = [identity(3) * sigma_pos ^ 2, zeros(3, 3); zeros(3, 3), identity(3) * sigma_vel ^ 2] .* T, R = diagonal(sigma_gps .^ 2), m = 3, n = 6, p = 6, x0 = x0) annotation(
        Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput q[4] annotation(
        Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput acc_body[3] annotation(
        Placement(visible = true, transformation(origin = {-120, 32}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 32}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput pos_ned[3] annotation(
        Placement(visible = true, transformation(origin = {-120, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -36}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput vel_ned[3] annotation(
        Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -98}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      RocketControl.Math.Blocks.resolve1 acc_ned annotation(
        Placement(visible = true, transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Math.Blocks.Vector.VectorConstant gravity(k = {0, 0, Modelica.Constants.g_n}) annotation(
        Placement(visible = true, transformation(origin = {-32, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      RocketControl.Math.Blocks.Vector.VectorAdd acc_inertial annotation(
        Placement(visible = true, transformation(origin = {10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput pos_est[3] annotation(
        Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput vel_est[3] annotation(
        Placement(visible = true, transformation(origin = {110, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanConstant enable annotation(
        Placement(visible = true, transformation(origin = {26, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.BooleanConstant booleanConstant(k = true) annotation(
        Placement(visible = true, transformation(origin = {20, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(q, acc_ned.q) annotation(
        Line(points = {{-120, 80}, {-76, 80}, {-76, 56}, {-62, 56}}, color = {0, 0, 127}, thickness = 0.5));
      connect(acc_body, acc_ned.x2) annotation(
        Line(points = {{-120, 32}, {-76, 32}, {-76, 44}, {-62, 44}}, color = {0, 0, 127}, thickness = 0.5));
      connect(gravity.v, acc_inertial.v1) annotation(
        Line(points = {{-20, 90}, {-12, 90}, {-12, 34}, {-2, 34}}, color = {0, 0, 127}, thickness = 0.5));
      connect(acc_ned.x1, acc_inertial.v2) annotation(
        Line(points = {{-38, 50}, {-22, 50}, {-22, 26}, {-2, 26}}, color = {0, 0, 127}, thickness = 0.5));
      connect(acc_inertial.vc, discreteKalmanFilter.u) annotation(
        Line(points = {{22, 30}, {32, 30}, {32, -6}, {40, -6}}, color = {0, 0, 127}, thickness = 0.5));
      connect(discreteKalmanFilter.x_est[1:3], pos_est) annotation(
        Line(points = {{60, 6}, {74, 6}, {74, 40}, {110, 40}}, color = {0, 0, 127}, thickness = 0.5));
      connect(discreteKalmanFilter.x_est[4:6], vel_est) annotation(
        Line(points = {{60, -6}, {74, -6}, {74, -20}, {110, -20}}, color = {0, 0, 127}, thickness = 0.5));
      connect(enable.y, discreteKalmanFilter.enable) annotation(
        Line(points = {{38, 78}, {50, 78}, {50, 10}}, color = {255, 0, 255}));
      connect(pos_ned, discreteKalmanFilter.y_meas[1:3]) annotation(
        Line(points = {{-120, -30}, {-20, -30}, {-20, 6}, {40, 6}}, color = {0, 0, 127}, thickness = 0.5));
      connect(vel_ned, discreteKalmanFilter.y_meas[4:6]) annotation(
        Line(points = {{-120, -80}, {-20, -80}, {-20, 6}, {40, 6}}, color = {0, 0, 127}, thickness = 0.5));
      connect(booleanConstant.y, discreteKalmanFilter.correct) annotation(
        Line(points = {{32, -62}, {50, -62}, {50, -10}}, color = {255, 0, 255}));
      annotation(
        Icon(graphics = {Text(origin = {-130, 70}, lineColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "q"), Text(origin = {-126, 6}, lineColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "acc"), Text(origin = {-126, -62}, lineColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "pos"), Text(origin = {-130, -128}, lineColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "vel"), Text(origin = {110, 18}, lineColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "pos_est"), Text(origin = {110, -64}, lineColor = {102, 102, 102}, extent = {{-30, 20}, {30, -20}}, textString = "vel_est"), Text(origin = {0, -76}, fillColor = {102, 102, 102}, extent = {{-100, 18}, {100, -18}}, textString = "pos/vel")}));
    end PositionEstimation;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Navigation;

  package Control

    model Control2Deflection
      final parameter Real M[4, 4] = [-0.5, 0, 0.5, 0; 0, 0.5, 0, -0.5; -0.25, -0.25, -0.25, -0.25; -0.25, 0.25, -0.25, 0.25] annotation(
        Evaluate = true);
      final parameter Real Minv[4, 4] = inv(M) annotation(
        Evaluate = true);
      Modelica.Blocks.Interfaces.RealInput u[3] annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput deflection[4] annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      deflection = Minv * cat(1, u, {0});
      annotation(
        Icon(graphics = {Text(extent = {{-100, 100}, {100, -100}}, textString = "c2f")}));
    end Control2Deflection;

    model Deflection2Control
      final parameter Real M[4, 4] = [-0.5, 0, 0.5, 0; 0, 0.5, 0, -0.5; -0.25, -0.25, -0.25, -0.25; -0.25, 0.25, -0.25, 0.25] annotation(
        Evaluate = true);
      Modelica.Blocks.Interfaces.RealInput u[4] annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput control[4] annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      control = M * u;
      annotation(
        Icon(graphics = {Text(extent = {{-100, 100}, {100, -100}}, textString = "f2c")}));
    end Deflection2Control;

    package LinearStateMatrices

      block RocketAndActuator
  extends Internal.PartialLinearStateMatrix;
        outer RocketControl.World.Atmosphere atmosphere;
        parameter Real CA0;
        parameter Real CA_a;
        parameter Real CA_b;
        parameter Real CA_dy;
        parameter Real CA_dp;
        parameter Real CA_dr;
        parameter Real CA_ds;
        parameter Real CN_a;
        parameter Real CN_dp;
        parameter Real CY_b;
        parameter Real CY_dy;
        parameter Real CLL_dr;
        parameter Real CLM_a;
        parameter Real CLM_dp;
        parameter Real CLN_b;
        parameter Real CLN_dy;
        parameter Real S;
        parameter Real c;
        parameter Real m;
        parameter Real Ix;
        parameter Real Is;
        parameter Real wa;
        SI.Velocity u;
        SI.Velocity v;
        SI.Velocity w;
        SI.AngularVelocity p;
        SI.AngularVelocity q;
        SI.AngularVelocity r;
        SI.Angle dys;
        SI.Angle dps;
        SI.Angle drs;
        SI.Density rho(displayUnit = "kg/m3");
      equation
        rho = atmosphere.density(-bus.x_est[3]);
        u = bus.v_est[1];
        v = bus.v_est[2];
        w = bus.v_est[3];
        p = bus.w_est[1];
        q = bus.w_est[2];
        r = bus.w_est[3];
        dys = bus.actuator_pos_meas[1];
        dps = bus.actuator_pos_meas[2];
        drs = bus.actuator_pos_meas[3];
        A = [-CA0 * S * rho * u / m, r - CA_b * S * rho * v / m, (-q) - CA_a * S * rho * w / m, 0, -w, v, 0, 0, 0; CY_b * S * rho * v / (2 * m) - r + CY_dy * S * dys * rho * u / m, CY_b * S * rho * u / (2 * m), p, w, 0, -u, CY_dy * S * rho * u ^ 2 / (2 * m), 0, 0; q - CN_a * S * rho * w / (2 * m) - CN_dp * S * dps * rho * u / m, -p, -CN_a * S * rho * u / (2 * m), -v, u, 0, 0, -CN_dp * S * rho * u ^ 2 / (2 * m), 0; CLL_dr * S * c * drs * rho * u / Ix, 0, 0, 0, 0, 0, 0, 0, CLL_dr * S * c * rho * u ^ 2 / (2 * Ix); CLM_a * S * c * rho * w / (2 * Is) + CLM_dp * S * c * dps * rho * u / Is, 0, CLM_a * S * c * rho * u / (2 * Is), r - Ix * r / Is, 0, p - Ix * p / Is, 0, CLM_dp * S * c * rho * u ^ 2 / (2 * Is), 0; CLN_b * S * c * rho * v / (2 * Is) + CLN_dy * S * c * dys * rho * u / Is, CLN_b * S * c * rho * u / (2 * Is), 0, Ix * q / Is - q, Ix * p / Is - p, 0, CLN_dy * S * c * rho * u ^ 2 / (2 * Is), 0, 0; 0, 0, 0, 0, 0, 0, -wa, 0, 0; 0, 0, 0, 0, 0, 0, 0, -wa, 0; 0, 0, 0, 0, 0, 0, 0, 0, -wa];
        B = [0, 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0; wa, 0, 0; 0, wa, 0; 0, 0, wa];
        annotation(
          Icon,
          Diagram);
      end RocketAndActuator;
    

      package OLD
        model SystemMatrices
          extends RocketControl.Math.Blocks.Matrix.Internal.MatrixIcon;
          parameter Real CA0;
          parameter Real CA_a;
          parameter Real CA_b;
          parameter Real CA_dy;
          parameter Real CA_dp;
          parameter Real CA_dr;
          parameter Real CA_ds;
          parameter Real CN_a;
          parameter Real CN_dp;
          parameter Real CY_b;
          parameter Real CY_dy;
          parameter Real CLL_dr;
          parameter Real CLM_a;
          parameter Real CLM_dp;
          parameter Real CLN_b;
          parameter Real CLN_dy;
          parameter Real S;
          parameter Real c;
          parameter Real m;
          parameter Real Ix;
          parameter Real Is;
          Modelica.Blocks.Interfaces.RealOutput A[5, 5] annotation(
            Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput B[5, 3] annotation(
            Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput ang_vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput rho annotation(
            Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Real u;
          Real v;
          Real w;
          Real p;
          Real q;
          Real r;
        equation
          u = vel[1];
          v = vel[2];
          w = vel[3];
          p = ang_vel[1];
          q = ang_vel[2];
          r = ang_vel[3];
          A = [CY_b * S * rho * u / (2 * m), p, w, 0, -u; -p, -CN_a * S * rho * u / (2 * m), -v, u, 0; 0, 0, 0, 0, 0; 0, CLM_a * S * c * rho * u / (2 * Is), r - Ix * r / Is, 0, p - Ix * p / Is; CLN_b * S * c * rho * u / (2 * Is), 0, Ix * q / Is - q, Ix * p / Is - p, 0];
          B = [CY_dy * S * rho * u ^ 2 / (2 * m), 0, 0; 0, -CN_dp * S * rho * u ^ 2 / (2 * m), 0; 0, 0, CLL_dr * S * c * rho * u ^ 2 / (2 * Ix); 0, CLM_dp * S * c * rho * u ^ 2 / (2 * Is), 0; CLN_dy * S * c * rho * u ^ 2 / (2 * Is), 0, 0];
          annotation(
            Icon(coordinateSystem(grid = {2, 0})));
          annotation(
            Icon(graphics = {Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "A,B"), Text(origin = {-79, 61}, textColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "v"), Text(origin = {-79, 1}, textColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "w"), Text(origin = {-66, -59}, textColor = {102, 102, 102}, extent = {{-34, 19}, {34, -19}}, textString = "rho"), Text(origin = {-2, -250}, textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {77, 49}, textColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "A"), Text(origin = {79, -47}, textColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "B")}),
            Diagram);
        end SystemMatrices;
      
        model RollHeadingOutput
          Modelica.Blocks.Interfaces.RealOutput C[2, 5] annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput q[4] annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Real a[3] "Euler angles";
        equation
          a = RocketControl.Math.quat2euler(q);
          C = [0, 0, 1, sin(a[3]) * tan(a[2]), cos(a[3]) * tan(a[2]); 0, 0, 0, sin(a[3]) / cos(a[2]), cos(a[3]) / cos(a[2])];
      // Roll rate
      // Yaw (heading) rate
          annotation(
            Icon(graphics = {Rectangle(lineColor = {255, 85, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {77, -1}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "C"), Text(origin = {-79, 1}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "q"), Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "C")}));
        end RollHeadingOutput;
      
        model SystemMatricesU
          extends RocketControl.Math.Blocks.Matrix.Internal.MatrixIcon;
          parameter Real CA0;
          parameter Real CA_a;
          parameter Real CA_b;
          parameter Real CA_dy;
          parameter Real CA_dp;
          parameter Real CA_dr;
          parameter Real CA_ds;
          parameter Real CN_a;
          parameter Real CN_dp;
          parameter Real CY_b;
          parameter Real CY_dy;
          parameter Real CLL_dr;
          parameter Real CLM_a;
          parameter Real CLM_dp;
          parameter Real CLN_b;
          parameter Real CLN_dy;
          parameter Real S;
          parameter Real c;
          parameter Real m;
          parameter Real Ix;
          parameter Real Is;
          Modelica.Blocks.Interfaces.RealOutput A[6, 6] annotation(
            Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput B[6, 3] annotation(
            Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput ang_vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput rho annotation(
            Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Real u;
          Real v;
          Real w;
          Real p;
          Real q;
          Real r;
        equation
          u = vel[1];
          v = vel[2];
          w = vel[3];
          p = ang_vel[1];
          q = ang_vel[2];
          r = ang_vel[3];
          A = [-CA0 * S * rho * u / m, r - CA_b * S * rho * v / m, (-q) - CA_a * S * rho * w / m, 0, -w, v; CY_b * S * rho * v / (2 * m) - r, CY_b * S * rho * u / (2 * m), p, w, 0, -u; q - CN_a * S * rho * w / (2 * m), -p, -CN_a * S * rho * u / (2 * m), -v, u, 0; 0, 0, 0, 0, 0, 0; CLM_a * S * c * rho * w / (2 * Is), 0, CLM_a * S * c * rho * u / (2 * Is), r - Ix * r / Is, 0, p - Ix * p / Is; CLN_b * S * c * rho * v / (2 * Is), CLN_b * S * c * rho * u / (2 * Is), 0, Ix * q / Is - q, Ix * p / Is - p, 0];
          B = [0, 0, 0; CY_dy * S * rho * u ^ 2 / (2 * m), 0, 0; 0, -CN_dp * S * rho * u ^ 2 / (2 * m), 0; 0, 0, CLL_dr * S * c * rho * u ^ 2 / (2 * Ix); 0, CLM_dp * S * c * rho * u ^ 2 / (2 * Is), 0; CLN_dy * S * c * rho * u ^ 2 / (2 * Is), 0, 0];
          annotation(
            Icon(coordinateSystem(grid = {2, 0})));
          annotation(
            Icon(graphics = {Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "A,B"), Text(origin = {-79, 61}, textColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "v"), Text(origin = {-79, 1}, textColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "w"), Text(origin = {-66, -59}, textColor = {102, 102, 102}, extent = {{-34, 19}, {34, -19}}, textString = "rho"), Text(origin = {-2, -250}, textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {77, 49}, textColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "A"), Text(origin = {79, -47}, textColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "B")}),
            Diagram);
        end SystemMatricesU;
      
        model OutputMatrixU
          Modelica.Blocks.Interfaces.RealInput q[4] annotation(
            Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput C[3, 6] annotation(
            Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput A[6, 6] annotation(
            Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 70}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput B[6, 3] annotation(
            Placement(visible = true, transformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput D[3, 3] annotation(
            Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        protected
          Real a[3];
        equation
          a = RocketControl.Math.quat2euler(q);
      // Outputs: Roll rate, x accel, y accel
          C = [0, 0, 0, 1, sin(a[3]) * tan(a[2]), cos(a[3]) * tan(a[2]); A[2:3, :]];
          D = [zeros(1, 3); B[2:3, :]];
          annotation(
            Icon(graphics = {Rectangle(lineColor = {255, 85, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "C"), Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {79, 43}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "C"), Text(origin = {-79, -59}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "q"), Text(origin = {79, -41}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "D"), Text(origin = {-79, 73}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "A"), Text(origin = {-79, 21}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "B")}));
        end OutputMatrixU;
      
        model RollOutput
          Modelica.Blocks.Interfaces.RealOutput C[1, 6] annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput q[4] annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Real a[3] "Euler angles";
        equation
          a = RocketControl.Math.quat2euler(q);
          C = [0, 0, 0, 1, sin(a[3]) * tan(a[2]), cos(a[3]) * tan(a[2])];
      // Roll rate
      // Yaw (heading) rate
          annotation(
            Icon(graphics = {Rectangle(lineColor = {255, 85, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {77, -1}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "C"), Text(origin = {-79, 1}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "q"), Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "C")}));
        end RollOutput;
      
        model SystemMatricesDer
          extends RocketControl.Math.Blocks.Matrix.Internal.MatrixIcon;
          parameter Real CA0;
          parameter Real CA_a;
          parameter Real CA_b;
          parameter Real CA_dy;
          parameter Real CA_dp;
          parameter Real CA_dr;
          parameter Real CA_ds;
          parameter Real CN_a;
          parameter Real CN_dp;
          parameter Real CY_b;
          parameter Real CY_dy;
          parameter Real CLL_dr;
          parameter Real CLM_a;
          parameter Real CLM_dp;
          parameter Real CLN_b;
          parameter Real CLN_dy;
          parameter Real S;
          parameter Real c;
          parameter Real m;
          parameter Real Ix;
          parameter Real Is;
          Modelica.Blocks.Interfaces.RealOutput A[15, 15] annotation(
            Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput B[15, 3] annotation(
            Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 90}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput ang_vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput rho annotation(
            Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Real u;
          Real v;
          Real w;
          Real p;
          Real q;
          Real r;
          Real u_d;
          Real v_d;
          Real w_d;
          Real p_d;
          Real q_d;
          Real r_d;
          Real dy;
          Real dp;
          Real dr;
          Modelica.Blocks.Interfaces.RealInput ang_acc[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput acc[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput delta[3] annotation(
            Placement(visible = true, transformation(origin = {-120, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        equation
          u = vel[1];
          v = vel[2];
          w = vel[3];
          p = ang_vel[1];
          q = ang_vel[2];
          r = ang_vel[3];
          u_d = acc[1];
          v_d = acc[2];
          w_d = acc[3];
          p_d = ang_acc[1];
          q_d = ang_acc[2];
          r_d = ang_acc[3];
          dy = delta[1];
          dp = delta[2];
          dr = delta[3];
      // States: [u v w ud vd wd p q r pd qd rd dy dp dr]
          A = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0; -CA0 * S * rho * u_d / m, r_d - CA_b * S * rho * v_d / m, (-q_d) - CA_a * S * rho * w_d / m, -CA0 * S * rho * u / m, r - CA_b * S * rho * v / m, (-q) - CA_a * S * rho * w / m, 0, -w_d, v_d, 0, -w, v, 0, 0, 0; CY_b * S * rho * v_d / (2 * m) - r_d + CY_dy * S * dy * rho * u_d / m, CY_b * S * rho * u_d / (2 * m), p_d, CY_b * S * rho * v / (2 * m) - r + CY_dy * S * dy * rho * u / m, CY_b * S * rho * u / (2 * m), p, w_d, 0, -u_d, w, 0, -u, CY_dy * S * rho * u * u_d / m, 0, 0; q_d - CN_a * S * rho * w_d / (2 * m) - CN_dp * S * dp * rho * u_d / m, -p_d, -CN_a * S * rho * u_d / (2 * m), q - CN_a * S * rho * w / (2 * m) - CN_dp * S * dp * rho * u / m, -p, -CN_a * S * rho * u / (2 * m), -v_d, u_d, 0, -v, u, 0, 0, -CN_dp * S * rho * u * u_d / m, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0; CLL_dr * S * c * dr * rho * u_d / Ix, 0, 0, CLL_dr * S * c * dr * rho * u / Ix, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, CLL_dr * S * c * rho * u * u_d / Ix; CLM_a * S * c * rho * w_d / (2 * Is) + CLM_dp * S * c * dp * rho * u_d / Is, 0, CLM_a * S * c * rho * u_d / (2 * Is), CLM_a * S * c * rho * w / (2 * Is) + CLM_dp * S * c * dp * rho * u / Is, 0, CLM_a * S * c * rho * u / (2 * Is), r_d - Ix * r_d / Is, 0, p_d - Ix * p_d / Is, r - Ix * r / Is, 0, p - Ix * p / Is, 0, CLM_dp * S * c * rho * u * u_d / Is, 0; CLN_b * S * c * rho * v_d / (2 * Is) + CLN_dy * S * c * dy * rho * u_d / Is, CLN_b * S * c * rho * u_d / (2 * Is), 0, CLN_b * S * c * rho * v / (2 * Is) + CLN_dy * S * c * dy * rho * u / Is, CLN_b * S * c * rho * u / (2 * Is), 0, Ix * q_d / Is - q_d, Ix * p_d / Is - p_d, 0, Ix * q / Is - q, Ix * p / Is - p, 0, CLN_dy * S * c * rho * u * u_d / Is, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
          B = [0, 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0; CY_dy * S * rho * u ^ 2 / (2 * m), 0, 0; 0, -CN_dp * S * rho * u ^ 2 / (2 * m), 0; 0, 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, CLL_dr * S * c * rho * u ^ 2 / (2 * Ix); 0, CLM_dp * S * c * rho * u ^ 2 / (2 * Is), 0; CLN_dy * S * c * rho * u ^ 2 / (2 * Is), 0, 0; 1, 0, 0; 0, 1, 0; 0, 0, 1];
          annotation(
            Icon(graphics = {Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "A,B"), Text(origin = {-79, 101}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "v"), Text(origin = {-79, 61}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "w"), Text(origin = {-66, -99}, lineColor = {102, 102, 102}, extent = {{-34, 19}, {34, -19}}, textString = "rho"), Text(origin = {-10, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {77, 49}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "A"), Text(origin = {79, -47}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "B")}),
            Diagram);
        end SystemMatricesDer;
      
        model OutputMatrixDer
          Modelica.Blocks.Interfaces.RealInput q[4] annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput C[2, 15] annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        protected
          Real a[3];
        equation
          a = RocketControl.Math.quat2euler(q);
      // Outputs: Roll rate, x accel, y accel
          C = [zeros(1, 6), 1, sin(a[3]) * tan(a[2]), cos(a[3]) * tan(a[2]), zeros(1, 6); 0, 0, 0, 0, 1, 0, zeros(1, 9)];
          annotation(
            Icon(graphics = {Rectangle(lineColor = {255, 85, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "C"), Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {79, -1}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "C"), Text(origin = {-79, 1}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "q")}));
        end OutputMatrixDer;
      
        model RollVnedOutput
          Modelica.Blocks.Interfaces.RealOutput C[3, 6] annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput q[4] annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Real a[3] "Euler angles";
          Real T[3, 3] "Body 2 NED matrix";
        equation
          a = RocketControl.Math.quat2euler(q);
          T = -transpose(Modelica.Mechanics.MultiBody.Frames.Quaternions.to_T(q));
          C = [0, 0, 0, 1, sin(a[3]) * tan(a[2]), cos(a[3]) * tan(a[2]); T[1, 1], T[1, 2], T[1, 3], 0, 0, 0; T[2, 1], T[2, 2], T[2, 3], 0, 0, 0];
          annotation(
            Icon(graphics = {Rectangle(lineColor = {255, 85, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {77, -1}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "C"), Text(origin = {-79, 1}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "q"), Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "C")}));
        end RollVnedOutput;
      
        model SystemMatricesQ
          extends RocketControl.Math.Blocks.Matrix.Internal.MatrixIcon;
          parameter Real CA0;
          parameter Real CA_a;
          parameter Real CA_b;
          parameter Real CA_dy;
          parameter Real CA_dp;
          parameter Real CA_dr;
          parameter Real CA_ds;
          parameter Real CN_a;
          parameter Real CN_dp;
          parameter Real CY_b;
          parameter Real CY_dy;
          parameter Real CLL_dr;
          parameter Real CLM_a;
          parameter Real CLM_dp;
          parameter Real CLN_b;
          parameter Real CLN_dy;
          parameter Real S;
          parameter Real c;
          parameter Real m;
          parameter Real Ix;
          parameter Real Is;
          Modelica.Blocks.Interfaces.RealOutput A[10, 10] annotation(
            Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput B[10, 3] annotation(
            Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput ang_vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 24}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput rho annotation(
            Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-122, -84}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput quat[4] annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -26}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Real u;
          Real v;
          Real w;
          Real p;
          Real q;
          Real r;
          Real qx;
          Real qy;
          Real qz;
          Real qw;
        equation
          u = vel[1];
          v = vel[2];
          w = vel[3];
          p = ang_vel[1];
          q = ang_vel[2];
          r = ang_vel[3];
          qx = quat[1];
          qy = quat[2];
          qz = quat[3];
          qw = quat[4];
          A = [-CA0 * S * rho * u / m, r - CA_b * S * rho * v / m, (-q) - CA_a * S * rho * w / m, 0, -w, v, 0, 0, 0, 0; CY_b * S * rho * v / (2 * m) - r, CY_b * S * rho * u / (2 * m), p, w, 0, -u, 0, 0, 0, 0; q - CN_a * S * rho * w / (2 * m), -p, -CN_a * S * rho * u / (2 * m), -v, u, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; CLM_a * S * c * rho * w / (2 * Is), 0, CLM_a * S * c * rho * u / (2 * Is), r - Ix * r / Is, 0, p - Ix * p / Is, 0, 0, 0, 0; CLN_b * S * c * rho * v / (2 * Is), CLN_b * S * c * rho * u / (2 * Is), 0, Ix * q / Is - q, Ix * p / Is - p, 0, 0, 0, 0, 0; 0, 0, 0, qw / 2, -qz / 2, qy / 2, 0, r / 2, -q / 2, p / 2; 0, 0, 0, qz / 2, qw / 2, -qx / 2, -r / 2, 0, p / 2, q / 2; 0, 0, 0, -qy / 2, qx / 2, qw / 2, q / 2, -p / 2, 0, r / 2; 0, 0, 0, -qx / 2, -qy / 2, -qz / 2, -p / 2, -q / 2, -r / 2, 0];
          B = [0, 0, 0; CY_dy * S * rho * u ^ 2 / (2 * m), 0, 0; 0, -CN_dp * S * rho * u ^ 2 / (2 * m), 0; 0, 0, CLL_dr * S * c * rho * u ^ 2 / (2 * Ix); 0, CLM_dp * S * c * rho * u ^ 2 / (2 * Is), 0; CLN_dy * S * c * rho * u ^ 2 / (2 * Is), 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0];
          annotation(
            Icon(graphics = {Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "A,B"), Text(origin = {-79, 81}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "v"), Text(origin = {-79, 27}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "w"), Text(origin = {-66, -83}, lineColor = {102, 102, 102}, extent = {{-34, 19}, {34, -19}}, textString = "rho"), Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {77, 49}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "A"), Text(origin = {79, -47}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "B"), Text(origin = {-79, -23}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "q")}),
            Diagram);
        end SystemMatricesQ;
      
        model OutputNedVel
          Modelica.Blocks.Interfaces.RealOutput C[3, 10] annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput ang_vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput quat[4] annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-122, -72}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Real u;
          Real v;
          Real w;
          Real p;
          Real q;
          Real r;
          Real qx;
          Real qy;
          Real qz;
          Real qw;
        equation
          u = vel[1];
          v = vel[2];
          w = vel[3];
          p = ang_vel[1];
          q = ang_vel[2];
          r = ang_vel[3];
          qx = quat[1];
          qy = quat[2];
          qz = quat[3];
          qw = quat[4];
          C = [qw ^ 2 + qx ^ 2 - qy ^ 2 - qz ^ 2, 2 * qx * qy - 2 * qw * qz, 2 * qw * qy + 2 * qx * qz, 0, 0, 0, 2 * qx * u + 2 * qy * v + 2 * qz * w, 2 * qx * v - 2 * qy * u + 2 * qw * w, 2 * qx * w - 2 * qw * v - 2 * qz * u, 2 * qw * u - 2 * qz * v + 2 * qy * w; 2 * qw * qz + 2 * qx * qy, qw ^ 2 - qx ^ 2 + qy ^ 2 - qz ^ 2, 2 * qy * qz - 2 * qw * qx, 0, 0, 0, 2 * qy * u - 2 * qx * v - 2 * qw * w, 2 * qx * u + 2 * qy * v + 2 * qz * w, 2 * qw * u - 2 * qz * v + 2 * qy * w, 2 * qz * u + 2 * qw * v - 2 * qx * w; 2 * qx * qz - 2 * qw * qy, 2 * qw * qx + 2 * qy * qz, qw ^ 2 - qx ^ 2 - qy ^ 2 + qz ^ 2, 0, 0, 0, 2 * qz * u + 2 * qw * v - 2 * qx * w, 2 * qz * v - 2 * qw * u - 2 * qy * w, 2 * qx * u + 2 * qy * v + 2 * qz * w, 2 * qx * v - 2 * qy * u + 2 * qw * w];
      // Roll rate
      // Yaw (heading) rate
          annotation(
            Icon(graphics = {Rectangle(lineColor = {255, 85, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {77, -1}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "C"), Text(origin = {-79, -75}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "q"), Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "C")}));
        end OutputNedVel;
      
        model SystemMatricesQSimpl
          extends RocketControl.Math.Blocks.Matrix.Internal.MatrixIcon;
          parameter Real CA0;
          parameter Real CA_a;
          parameter Real CA_b;
          parameter Real CA_dy;
          parameter Real CA_dp;
          parameter Real CA_dr;
          parameter Real CA_ds;
          parameter Real CN_a;
          parameter Real CN_dp;
          parameter Real CY_b;
          parameter Real CY_dy;
          parameter Real CLL_dr;
          parameter Real CLM_a;
          parameter Real CLM_dp;
          parameter Real CLN_b;
          parameter Real CLN_dy;
          parameter Real S;
          parameter Real c;
          parameter Real m;
          parameter Real Ix;
          parameter Real Is;
          Modelica.Blocks.Interfaces.RealOutput A[9, 9] annotation(
            Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput B[9, 3] annotation(
            Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput ang_vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 24}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput rho annotation(
            Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-122, -84}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput quat[4] annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -26}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Real u;
          Real v;
          Real w;
          Real p;
          Real q;
          Real r;
          Real qx;
          Real qy;
          Real qz;
          Real qw;
        equation
          u = vel[1];
          v = vel[2];
          w = vel[3];
          p = ang_vel[1];
          q = ang_vel[2];
          r = ang_vel[3];
          qx = quat[1];
          qy = quat[2];
          qz = quat[3];
          qw = quat[4];
          A = [-CA0 * S * rho * u / m, r - CA_b * S * rho * v / m, (-q) - CA_a * S * rho * w / m, 0, -w, v, 0, 0, 0; CY_b * S * rho * v / (2 * m) - r, CY_b * S * rho * u / (2 * m), p, w, 0, -u, 0, 0, 0; q - CN_a * S * rho * w / (2 * m), -p, -CN_a * S * rho * u / (2 * m), -v, u, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0; CLM_a * S * c * rho * w / (2 * Is), 0, CLM_a * S * c * rho * u / (2 * Is), r - Ix * r / Is, 0, p - Ix * p / Is, 0, 0, 0; CLN_b * S * c * rho * v / (2 * Is), CLN_b * S * c * rho * u / (2 * Is), 0, Ix * q / Is - q, Ix * p / Is - p, 0, 0, 0, 0; 0, 0, 0, 1 / 2, -qz / 2, qy / 2, 0, r / 2, -q / 2; 0, 0, 0, qz / 2, 1 / 2, -qx / 2, -r / 2, 0, p / 2; 0, 0, 0, -qy / 2, qx / 2, 1 / 2, q / 2, -p / 2, 0];
          B = [0, 0, 0; CY_dy * S * rho * u ^ 2 / (2 * m), 0, 0; 0, -CN_dp * S * rho * u ^ 2 / (2 * m), 0; 0, 0, CLL_dr * S * c * rho * u ^ 2 / (2 * Ix); 0, CLM_dp * S * c * rho * u ^ 2 / (2 * Is), 0; CLN_dy * S * c * rho * u ^ 2 / (2 * Is), 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0];
          annotation(
            Icon(graphics = {Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "A,B"), Text(origin = {-79, 81}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "v"), Text(origin = {-79, 27}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "w"), Text(origin = {-66, -83}, lineColor = {102, 102, 102}, extent = {{-34, 19}, {34, -19}}, textString = "rho"), Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {77, 49}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "A"), Text(origin = {79, -47}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "B"), Text(origin = {-79, -23}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "q")}),
            Diagram);
        end SystemMatricesQSimpl;
      
        model OutputNedVelSimpl
          Modelica.Blocks.Interfaces.RealOutput C[2, 9] annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput ang_vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput quat[4] annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-122, -72}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Real u;
          Real v;
          Real w;
          Real p;
          Real q;
          Real r;
          Real qx;
          Real qy;
          Real qz;
          Real qw;
        equation
          u = vel[1];
          v = vel[2];
          w = vel[3];
          p = ang_vel[1];
          q = ang_vel[2];
          r = ang_vel[3];
          qx = quat[1];
          qy = quat[2];
          qz = quat[3];
          qw = quat[4];
          C = -[-2 * qz, 1, 2 * qx, 0, 0, 0, 2 * w, 0, -2 * u; 2 * qy, -2 * qx, 1, 0, 0, 0, -2 * v, 2 * u, 0];
      // Roll rate
      // Yaw (heading) rate
          annotation(
            Icon(graphics = {Rectangle(lineColor = {255, 85, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {77, -1}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "C"), Text(origin = {-79, -75}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "q"), Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "C")}));
        end OutputNedVelSimpl;
      
        model RollAndRatesOutput
          Modelica.Blocks.Interfaces.RealOutput C[3, 6] annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput q[4] annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Real a[3] "Euler angles";
        equation
          a = RocketControl.Math.quat2euler(q);
          C = [0, 0, 0, 1, 0, 0; 0, 0, 0, 0, 1, 0; 0, 0, 0, 0, 0, 1];
      //0, 0, 0, 1, sin(a[3]) * tan(a[2]), cos(a[3]) * tan(a[2]);
          annotation(
            Icon(graphics = {Rectangle(lineColor = {255, 85, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {77, -1}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "C"), Text(origin = {-79, 1}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "q"), Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "C")}));
        end RollAndRatesOutput;
      
        block RocketAndActuator
          extends RocketControl.Math.Blocks.Matrix.Internal.MatrixIcon;
          outer RocketControl.World.Atmosphere atmosphere;
          parameter Real CA0;
          parameter Real CA_a;
          parameter Real CA_b;
          parameter Real CA_dy;
          parameter Real CA_dp;
          parameter Real CA_dr;
          parameter Real CA_ds;
          parameter Real CN_a;
          parameter Real CN_dp;
          parameter Real CY_b;
          parameter Real CY_dy;
          parameter Real CLL_dr;
          parameter Real CLM_a;
          parameter Real CLM_dp;
          parameter Real CLN_b;
          parameter Real CLN_dy;
          parameter Real S;
          parameter Real c;
          parameter Real m;
          parameter Real Ix;
          parameter Real Is;
          parameter Real wa;
          Modelica.Blocks.Interfaces.RealOutput A[9, 9] annotation(
            Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput B[9, 3] annotation(
            Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          SI.Velocity u;
          SI.Velocity v;
          SI.Velocity w;
          SI.AngularVelocity p;
          SI.AngularVelocity q;
          SI.AngularVelocity r;
          SI.Angle dys;
          SI.Angle dps;
          SI.Angle drs;
          SI.Density rho(displayUnit = "kg/m3");
          Modelica.Blocks.Interfaces.RealInput vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 90}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 92}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput x_est[3] annotation(
            Placement(visible = true, transformation(origin = {-120, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput ang_vel[3] annotation(
            Placement(visible = true, transformation(origin = {-120, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput control_cmd[3] annotation(
            Placement(visible = true, transformation(origin = {-120, -92}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        equation
          rho = atmosphere.density(-x_est[3]);
          u = vel[1];
          v = vel[2];
          w = vel[3];
          p = ang_vel[1];
          q = ang_vel[2];
          r = ang_vel[3];
          dys = control_cmd[1];
          dps = control_cmd[2];
          drs = control_cmd[3];
          A = [-CA0 * S * rho * u / m, r - CA_b * S * rho * v / m, (-q) - CA_a * S * rho * w / m, 0, -w, v, 0, 0, 0; CY_b * S * rho * v / (2 * m) - r + CY_dy * S * dys * rho * u / m, CY_b * S * rho * u / (2 * m), p, w, 0, -u, CY_dy * S * rho * u ^ 2 / (2 * m), 0, 0; q - CN_a * S * rho * w / (2 * m) - CN_dp * S * dps * rho * u / m, -p, -CN_a * S * rho * u / (2 * m), -v, u, 0, 0, -CN_dp * S * rho * u ^ 2 / (2 * m), 0; CLL_dr * S * c * drs * rho * u / Ix, 0, 0, 0, 0, 0, 0, 0, CLL_dr * S * c * rho * u ^ 2 / (2 * Ix); CLM_a * S * c * rho * w / (2 * Is) + CLM_dp * S * c * dps * rho * u / Is, 0, CLM_a * S * c * rho * u / (2 * Is), r - Ix * r / Is, 0, p - Ix * p / Is, 0, CLM_dp * S * c * rho * u ^ 2 / (2 * Is), 0; CLN_b * S * c * rho * v / (2 * Is) + CLN_dy * S * c * dys * rho * u / Is, CLN_b * S * c * rho * u / (2 * Is), 0, Ix * q / Is - q, Ix * p / Is - p, 0, CLN_dy * S * c * rho * u ^ 2 / (2 * Is), 0, 0; 0, 0, 0, 0, 0, 0, -wa, 0, 0; 0, 0, 0, 0, 0, 0, 0, -wa, 0; 0, 0, 0, 0, 0, 0, 0, 0, -wa];
          B = [0, 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0; 0, 0, 0; wa, 0, 0; 0, wa, 0; 0, 0, wa];
          annotation(
            Icon(graphics = {Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "A,B"), Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {77, 49}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "A"), Text(origin = {79, -47}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "B")}),
            Diagram);
        end RocketAndActuator;
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end OLD;

      package Internal
        block PartialLinearStateMatrix
        Modelica.Blocks.Interfaces.RealOutput B annotation(
            Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput A annotation(
            Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
            Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        equation

          annotation(
            Icon(graphics = {Rectangle(lineColor = {255, 85, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {0, 4}, extent = {{-80, 68}, {80, -68}}, textString = "A,B"), Text(origin = {105, 21}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "A"), Text(origin = {105, -79}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "B"), Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
        end PartialLinearStateMatrix;
        annotation(
          Icon(coordinateSystem(grid = {2, 0})));
      end Internal;
      annotation(
        Icon);
    end LinearStateMatrices;

    package OLD
    model Autopilot
      parameter NonSI.Angle_deg fin_max_angle;
      parameter NonSI.Angle_deg fin_min_angle = -fin_max_angle;
      parameter Real CNalpha;
      parameter Real CNdp;
      parameter Real CYbeta;
      parameter Real CYdy;
      parameter Real CLLdr;
      parameter Real CLMalpha;
      parameter Real CLMdp;
      parameter Real CLNbeta;
      parameter Real CLNdy;
      parameter SI.Area S;
      parameter SI.Length c;
      NonSI.Angle_deg angles[5];
      NonSI.Angle_deg delta[4];
      NonSI.Angle_deg delta_sat[4];
      NonSI.Angle_deg deflection_nonsat[4];
      SI.Force expected_ra[5];
      Modelica.Blocks.Interfaces.RealInput Fn annotation(
        Placement(visible = true, transformation(origin = {-100, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Fy annotation(
        Placement(visible = true, transformation(origin = {-100, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Ml annotation(
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput q annotation(
        Placement(visible = true, transformation(origin = {-100, -58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput deflection[4] annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    protected
      final parameter Real KC[5, 5] = [CNalpha, 0, 0, CNdp, 0; 0, CYbeta, CYdy, 0, 0; 0, 0, 0, 0, c * CLLdr; c * CLMalpha, 0, 0, c * CLMdp, 0; 0, c * CLNbeta, c * CLNdy, 0, 0];
      final parameter Real M[4, 4] = [-0.5, 0, 0.5, 0; 0, 0.5, 0, -0.5; -0.25, -0.25, -0.25, -0.25; -0.25, 0.25, -0.25, 0.25] annotation(
        Evaluate = true);
      final parameter Real Minv[4, 4] = inv(M) annotation(
        Evaluate = true);
      Real R[5];
      Real K[5, 5];
    equation
      R = {-Fn, Fy, Ml, 0, 0};
      K = q * S * KC;
      if noEvent(q > 10) then
        angles = Modelica.Math.Matrices.solve(K, R);
        delta = {angles[3], angles[4], angles[5], 0};
        deflection_nonsat = Minv * delta;
      else
        angles = {0, 0, 0, 0, 0};
        delta = {0, 0, 0, 0};
        deflection_nonsat = {0, 0, 0, 0};
      end if;
      for i in 1:4 loop
        deflection[i] = max(fin_min_angle, min(deflection_nonsat[i], fin_max_angle));
      end for;
      delta_sat = M * deflection;
      expected_ra = K * {angles[1], angles[2], delta_sat[1], delta_sat[2], delta_sat[3]};
      annotation(
        Icon(graphics = {Text(origin = {-70, 80}, lineColor = {88, 88, 88}, extent = {{-30, 20}, {30, -20}}, textString = "Fn"), Text(origin = {-70, 40}, lineColor = {88, 88, 88}, extent = {{-30, 20}, {30, -20}}, textString = "Fy"), Text(origin = {-70, -80}, lineColor = {88, 88, 88}, extent = {{-30, 20}, {30, -20}}, textString = "q"), Text(origin = {108, -25}, lineColor = {88, 88, 88}, extent = {{-30, 19}, {30, -19}}, textString = "fin"), Ellipse(fillColor = {76, 114, 124}, fillPattern = FillPattern.Sphere, extent = {{60, 60}, {-60, -60}}, endAngle = 360), Text(origin = {-70, 0}, lineColor = {88, 88, 88}, extent = {{-30, 20}, {30, -20}}, textString = "Ml"), Text(origin = {0, -130}, lineColor = {0, 0, 255}, extent = {{-160, 30}, {160, -30}}, textString = "%name")}));
    end Autopilot;
    model FinSaturation
      parameter NonSI.Angle_deg def_max = 10;
      Modelica.Blocks.Interfaces.RealInput fin[4] annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput fin_sat[4] annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Nonlinear.Limiter limiter(uMax = def_max) annotation(
        Placement(visible = true, transformation(origin = {0, 48}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Blocks.Nonlinear.Limiter limiter1(uMax = def_max) annotation(
        Placement(visible = true, transformation(origin = {0, 18}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Blocks.Nonlinear.Limiter limiter2(uMax = def_max) annotation(
        Placement(visible = true, transformation(origin = {0, -14}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Blocks.Nonlinear.Limiter limiter3(uMax = def_max) annotation(
        Placement(visible = true, transformation(origin = {0, -40}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    equation
      connect(fin[1], limiter.u) annotation(
        Line(points = {{-120, 0}, {-40, 0}, {-40, 48}, {-8, 48}}, color = {0, 0, 127}));
      connect(fin[2], limiter1.u) annotation(
        Line(points = {{-120, 0}, {-40, 0}, {-40, 18}, {-7, 18}}, color = {0, 0, 127}));
      connect(fin[3], limiter2.u) annotation(
        Line(points = {{-120, 0}, {-40, 0}, {-40, -14}, {-8, -14}}, color = {0, 0, 127}));
      connect(fin[4], limiter3.u) annotation(
        Line(points = {{-120, 0}, {-40, 0}, {-40, -40}, {-8, -40}}, color = {0, 0, 127}));
      connect(limiter3.y, fin_sat[4]) annotation(
        Line(points = {{6, -40}, {40, -40}, {40, 0}, {110, 0}}, color = {0, 0, 127}));
      connect(limiter2.y, fin_sat[3]) annotation(
        Line(points = {{6, -14}, {40, -14}, {40, 0}, {110, 0}}, color = {0, 0, 127}));
      connect(limiter1.y, fin_sat[2]) annotation(
        Line(points = {{7, 18}, {40, 18}, {40, 0}, {110, 0}}, color = {0, 0, 127}));
      connect(limiter.y, fin_sat[1]) annotation(
        Line(points = {{6, 48}, {40, 48}, {40, 0}, {110, 0}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{0, 90}, {-8, 68}, {8, 68}, {0, 90}}), Line(visible = false, points = {{50, 70}, {80, 70}}, color = {255, 0, 0}), Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Line(points = {{0, -90}, {0, 68}}, color = {192, 192, 192}), Line(visible = false, points = {{-80, -70}, {-50, -70}}, color = {255, 0, 0}), Line(points = {{-80, -70}, {-50, -70}, {50, 70}, {80, 70}}), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{90, 0}, {68, -8}, {68, 8}, {90, 0}}), Line(points = {{-90, 0}, {68, 0}}, color = {192, 192, 192}), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(extent = {{-150, -150}, {150, -110}}, textString = "uMax=%def_max")}));
    end FinSaturation;
    model AngleFromTarget
      Modelica.Blocks.Interfaces.RealInput v[3] annotation(
        Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput vt[3] annotation(
        Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput angle(final unit = "rad", final quantity = "Angle", final displayUnit = "deg") annotation(
        Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput angle_vec[3] annotation(
        Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    protected
      Real cp[3];
    equation
      angle = acos(v * vt / (norm(v) * norm(vt)));
      cp = cross(v, vt);
      angle_vec = cp / norm(cp) * angle;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end AngleFromTarget;
    model AngularRateToTarget
      parameter Real k;
      Modelica.Blocks.Interfaces.RealInput vt_ned[3] annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput v_ned[3] annotation(
        Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput qr[2](each final unit = "rad/s", each final quantity = "AngularSpeed", each displayUnit = "deg/s") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput q[4] annotation(
        Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      RocketControl.Components.Blocks.ned2body vt annotation(
        Placement(visible = true, transformation(origin = {-70, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SI.Angle theta(displayUnit = "deg");
      Real dir[3];
      Real norm_dir;
      RocketControl.Components.Blocks.ned2body v annotation(
        Placement(visible = true, transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      if norm(v.x_b) < 1e-6 then
        theta = 0;
        dir = zeros(3);
        norm_dir = 0;
        qr = zeros(2);
      else
        theta = acos(v.x_b * vt.x_b / (norm(v.x_b) * norm(vt.x_b)));
        dir = cross(v.x_b, vt.x_b);
        norm_dir = norm(dir);
        if norm_dir < 1e-6 then
          qr = {0, 0};
        else
          qr = dir[2:3] / norm_dir * theta * k;
        end if;
      end if;
      connect(q, vt.q_bw) annotation(
        Line(points = {{-120, -60}, {-92, -60}, {-92, -16}, {-82, -16}}, color = {0, 0, 127}, thickness = 0.5));
      connect(vt_ned, vt.x_w) annotation(
        Line(points = {{-120, 0}, {-92, 0}, {-92, -4}, {-82, -4}}, color = {0, 0, 127}, thickness = 0.5));
      connect(v_ned, v.x_w) annotation(
        Line(points = {{-120, 60}, {-92, 60}, {-92, 56}, {-82, 56}}, color = {0, 0, 127}, thickness = 0.5));
      connect(q, v.q_bw) annotation(
        Line(points = {{-120, -60}, {-92, -60}, {-92, 44}, {-82, 44}}, color = {0, 0, 127}, thickness = 0.5));
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end AngularRateToTarget;
    model ProportionalHeadingRate
      parameter Real k;
      Modelica.Blocks.Interfaces.RealInput track_ref annotation(
        Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput hr(final unit = "rad/s", final quantity = "AngularSpeed", displayUnit = "deg/s") annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      SI.Angle delta_track(displayUnit = "deg");
      Real track_vers[3];
      Real track_cp[3];
      Real v_horiz[3];
      Modelica.Blocks.Interfaces.RealInput v_ned[3] annotation(
        Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      track_vers = {cos(track_ref), sin(track_ref), 0};
      v_horiz = cat(1, v_ned[1:2], {0});
      if norm(v_ned) < 1e-6 then
        delta_track = 0;
        hr = 0;
      else
        track_cp = cross(v_ned, track_vers);
        if norm(track_cp) < 1e-6 then
          delta_track = 0;
        else
          delta_track = acos(v_horiz * track_vers / norm(v_horiz)) * sign(track_cp[3]);
        end if;
        hr = delta_track * k;
      end if;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end ProportionalHeadingRate;
    block LinearFeedforward
      extends RocketControl.Components.Interfaces.PartialConditionalEnablePort;
      parameter Integer n(min = 1) = 1;
      parameter Integer m(min = 1) = 1;
      parameter Integer p(min = 1) = 1;
      parameter Integer nk(min = 1) = n;
      Modelica.Blocks.Interfaces.RealInput A[n, n] annotation(
        Placement(visible = true, transformation(origin = {-120, 98}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput B[n, m] annotation(
        Placement(visible = true, transformation(origin = {-120, 70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput u_ff[m] annotation(
        Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput C[p, n] annotation(
        Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput r[p] annotation(
        Placement(visible = true, transformation(origin = {-120, -78}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput x_eq[n] annotation(
        Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput u[m] annotation(
        Placement(visible = true, transformation(origin = {-120, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput K[m, nk] annotation(
        Placement(visible = true, transformation(origin = {-120, 10}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    protected
      final parameter Real D[p, m] = zeros(p, m);
      Real M[n + p, n + m];
      Real N[n + m, p];
    equation
      if enable then
        M = [A, B; C, D];
        N = Modelica.Math.Matrices.leastSquares2(M, [zeros(n, p); identity(p)]);
        x_eq = N[1:n, :] * r;
        u_ff = u + K * cat(1, x_eq, zeros(nk - n)) + N[n + 1:end, :] * r;
      else
        M = zeros(n + p, n + m);
        N = zeros(n + m, p);
        x_eq = zeros(n);
        u_ff = zeros(m);
      end if;
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end LinearFeedforward;
    model ContinuousLQRWithFF
      extends RocketControl.Components.Interfaces.PartialConditionalEnablePort;
      parameter Integer n(min = 1) = 1 annotation(
        Evaluate = true);
      parameter Integer m(min = 1) = 1 annotation(
        Evaluate = true);
      parameter Integer p(min = 1) = 1 annotation(
        Evaluate = true);
      parameter Real g = -1;
      Modelica.Blocks.Interfaces.RealInput A[n, n] annotation(
        Placement(visible = true, transformation(origin = {-120, 90}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput B[n, m] annotation(
        Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Q[n, n] annotation(
        Placement(visible = true, transformation(origin = {-120, 10}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput R[m, m] annotation(
        Placement(visible = true, transformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput u[m] annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput x[n] annotation(
        Placement(visible = true, transformation(origin = {-120, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Real P[n, n];
      Integer niter;
      Real K[m, n];
      Modelica.Blocks.Interfaces.RealInput r[p] annotation(
        Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput C[p, n] annotation(
        Placement(visible = true, transformation(origin = {-120, 36}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      RocketControl.GNC.Control.LinearFeedforward linearFeedforward(m = m, n = n, p = p) annotation(
        Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      if enable then
        (P, niter) = RocketControl.Math.sdaCare(A, B, R, Q, g);
        K = Modelica.Math.Matrices.solve2(R, transpose(B) * P);
        u = (-K * (x - linearFeedforward.x_eq)) + linearFeedforward.u_ff;
      else
        P = zeros(n, n);
        K = zeros(m, n);
        niter = 0;
        u = zeros(m);
      end if;
      connect(A, linearFeedforward.A) annotation(
        Line(points = {{-120, 90}, {-60, 90}, {-60, 38}, {-42, 38}}, color = {0, 0, 127}, thickness = 0.5));
      connect(B, linearFeedforward.B) annotation(
        Line(points = {{-120, 60}, {-64, 60}, {-64, 34}, {-42, 34}}, color = {0, 0, 127}, thickness = 0.5));
      connect(C, linearFeedforward.C) annotation(
        Line(points = {{-120, 36}, {-68, 36}, {-68, 30}, {-42, 30}}, color = {0, 0, 127}, thickness = 0.5));
      connect(r, linearFeedforward.r) annotation(
        Line(points = {{-120, -80}, {-50, -80}, {-50, 22}, {-42, 22}}, color = {0, 0, 127}, thickness = 0.5));
      annotation(
        Icon(graphics = {Polygon(fillColor = {247, 219, 255}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-100, 100}, {-100, -80}, {-100, -100}, {70, -100}, {100, -80}, {100, 80}, {70, 100}, {-80, 100}}), Text(origin = {-2, 1}, extent = {{-60, 59}, {60, -59}}, textString = "LQ"), Text(origin = {-80, 120}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "A"), Text(origin = {-80, 80}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "B"), Text(origin = {-80, 0}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "Q"), Text(origin = {-80, -40}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "R"), Text(origin = {80, 0}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "u"), Rectangle(origin = {-100, -100}, fillColor = {217, 255, 215}, fillPattern = FillPattern.Solid, extent = {{-40, 40}, {40, -40}}), Text(origin = {-80, -80}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "x"), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {-80, -120}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "r"), Text(origin = {-80, 40}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "C")}));
    end ContinuousLQRWithFF;
    
      annotation(
        Icon(coordinateSystem(grid = {2, 0})));
    end OLD;
    annotation(
      Icon(coordinateSystem(grid = {2, 0})));
  end Control;

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
