within RocketControl.GNC.Control;

block BodyVelocityControlDiscrete
  outer World.SimOptions opt;
  parameter Real Qvec[:] = {0, 20, 20, 100, 0, 0, 0.1, 0.1, 0.1} * 1;
  parameter Real Rvec[:] = {1, 1, 0.4} * 50;
  RocketControl.GNC.Control.LinearStateMatrices.RocketAndActuator rocket(CA0 = 0.4200, CA_a = -0.1725, CA_b = -0.1725, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22, wa = 13) annotation(
    Placement(visible = true, transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Matrices.MatrixConstant Q(n = size(Qvec, 1), val = diagonal(Qvec))  annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Matrices.MatrixConstant R(n = 3, val = diagonal(Rvec))  annotation(
    Placement(visible = true, transformation(origin = {-50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
    Placement(visible = true, transformation(origin = {56, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput vel_error[3] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConcatenate x_lq(n2 = 3)  annotation(
    Placement(visible = true, transformation(origin = {-50, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConstant vectorConstant1(k = {0, 0, 0, 0}, n = 4)  annotation(
    Placement(visible = true, transformation(origin = {40, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.DiscreteLQR discreteLQR(dt = opt.samplePeriodMs / 1000, m = 3, n = 9, s = 1, useEnablePort = true)  annotation(
    Placement(visible = true, transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConcatenate vectorConcatenate(n1 = 6, n2 = 3) annotation(
    Placement(visible = true, transformation(origin = {-8, -78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Blocks.Math.Vectors.VectorSplit vectorSplit(n = 4, s = 3)  annotation(
    Placement(visible = true, transformation(origin = {-58, -94}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(bus, rocket.bus) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 50}, {-60, 50}}, thickness = 0.5));
  connect(vel_error, x_lq.v1) annotation(
    Line(points = {{-120, 0}, {-76, 0}, {-76, -56}, {-62, -56}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.w_est, x_lq.v2) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, -64}, {-62, -64}}, thickness = 0.5));
  connect(bus.control_enable, discreteLQR.enable) annotation(
    Line(points = {{100, 0}, {100, 100}, {10, 100}, {10, 9}}, color = {255, 0, 255}));
  connect(rocket.A, discreteLQR.A) annotation(
    Line(points = {{-38, 56}, {-22, 56}, {-22, 8}, {-2, 8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(rocket.B, discreteLQR.B) annotation(
    Line(points = {{-38, 46}, {-26, 46}, {-26, 4}, {-2, 4}}, color = {0, 0, 127}, thickness = 0.5));
  connect(Q.k, discreteLQR.Q) annotation(
    Line(points = {{-38, 0}, {-2, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(R.k, discreteLQR.R) annotation(
    Line(points = {{-38, -30}, {-16, -30}, {-16, -4}, {-2, -4}}, color = {0, 0, 127}, thickness = 0.5));
  connect(discreteLQR.u, control2Deflection.u[1:3]) annotation(
    Line(points = {{21, 0}, {44, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(control2Deflection.deflection, bus.fin_setpoint) annotation(
    Line(points = {{67, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(x_lq.vc, vectorConcatenate.v1) annotation(
    Line(points = {{-38, -60}, {-28, -60}, {-28, -74}, {-20, -74}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorConcatenate.vc, discreteLQR.x) annotation(
    Line(points = {{4, -78}, {12, -78}, {12, -22}, {-12, -22}, {-12, -8}, {-2, -8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorSplit.va, vectorConcatenate.v2) annotation(
    Line(points = {{-46, -90}, {-38, -90}, {-38, -82}, {-20, -82}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.control_position_meas, vectorSplit.v) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, -94}, {-70, -94}}, thickness = 0.5));
  annotation(
    Icon(graphics = {Rectangle(fillColor = {255, 242, 254}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 12), Polygon(origin = {-72, -22}, fillColor = {91, 91, 91}, fillPattern = FillPattern.Solid, points = {{-2, -12}, {8, -22}, {22, 12}, {12, 22}, {-22, 8}, {-2, -12}}), Polygon(origin = {-41, 9}, fillColor = {155, 183, 193}, fillPattern = FillPattern.Solid, points = {{-43, -33}, {25, 35}, {45, 45}, {35, 25}, {-33, -43}, {-37, -39}, {-43, -33}}), Line(origin = {24.4844, 12.5364}, points = {{-56, -7}, {56, 7}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 12), Line(origin = {30.3844, 41.6364}, points = {{-15.9035, 23.9035}, {0.0964559, 17.9035}, {8.09646, 7.90354}, {14.0965, -4.09646}, {16.0965, -24.0965}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 21), Text(origin = {-1, -68}, extent = {{-77, 18}, {77, -18}}, textString = "vtrack"), Text(origin = {2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end BodyVelocityControlDiscrete;
