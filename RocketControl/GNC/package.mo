within RocketControl;

package GNC
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
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end GNC;
