within RocketControl.GNC.Control;

block AngularRateControl
  outer World.SimOptions opt;
  parameter Real Qvec[:] = {0, 1, 1, 0, 0, 0, 0, 0, 0, 100,100,100} * 0.01;
  parameter Real Rvec[:] = {1, 1, 1} * 300;
  RocketControl.GNC.Control.LinearStateMatrices.RocketAndActuator rocket(CA0 = 0.4200, CA_a = -0.1725, CA_b = -0.1725, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLL_p = -19, CLM_a = -37.2959, CLM_dp = 21.8445, CLM_q = -1813, CLN_b = 37.2959, CLN_dy = 21.8445, CLN_r = -1813, CN_a = 24.0744, CN_dp = 3.4045, CN_q = 154, CY_b = -24.0744, CY_dy = 3.4045, CY_r = 154, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22, wa = 13) annotation(
    Placement(visible = true, transformation(origin = {-82, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Matrices.MatrixConstant Q(n = size(Qvec, 1), val = diagonal(Qvec))  annotation(
    Placement(visible = true, transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Matrices.MatrixConstant R(n = 3, val = diagonal(Rvec))  annotation(
    Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
    Placement(visible = true, transformation(origin = {70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.DiscreteLQR discreteLQR(dt = opt.samplePeriodMs / 1000, m = 3, n = 12, s = 1, useEnablePort = true)  annotation(
    Placement(visible = true, transformation(origin = {40, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput p_err_int annotation(
    Placement(visible = true, transformation(origin = {-120, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConcatenate vectorConcatenate1(n1 = 1, n2 = 2) annotation(
    Placement(visible = true, transformation(origin = {-54, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AugmentIntegratorState augmentIntegratorState(kint = 1,m = 3, n = 9, p = 3)  annotation(
    Placement(visible = true, transformation(origin = {-32, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.LinearOutputMatrices.AngularRatesOutput angularRatesOutput  annotation(
    Placement(visible = true, transformation(origin = {-82, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput qr_err_int[2] annotation(
    Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 42}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConcatenate x_1_6(n2 = 3) annotation(
    Placement(visible = true, transformation(origin = {-68, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Trasformations.ned2body v_body annotation(
    Placement(visible = true, transformation(origin = {-56, 132}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  RocketControl.Blocks.Math.Vectors.VectorConcatenate vectorConcatenate(n1 = 6, n2 = 3) annotation(
    Placement(visible = true, transformation(origin = {-28, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConcatenate vectorConcatenate2(n1 = 9, n2 = 3) annotation(
    Placement(visible = true, transformation(origin = {14, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(bus, rocket.bus) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 86}, {-92, 86}}, thickness = 0.5));
  connect(Q.k, discreteLQR.Q) annotation(
    Line(points = {{-39, 40}, {-20, 40}, {-20, 50}, {28, 50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(R.k, discreteLQR.R) annotation(
    Line(points = {{-39, 10}, {-16, 10}, {-16, 46}, {28, 46}}, color = {0, 0, 127}, thickness = 0.5));
  connect(discreteLQR.u, control2Deflection.u[1:3]) annotation(
    Line(points = {{51, 50}, {58, 50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(control2Deflection.deflection, bus.fin_setpoint) annotation(
    Line(points = {{81, 50}, {83.5, 50}, {83.5, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(rocket.A, augmentIntegratorState.A) annotation(
    Line(points = {{-70, 92}, {-58, 92}, {-58, 86}, {-44, 86}}, color = {0, 0, 127}, thickness = 0.5));
  connect(rocket.B, augmentIntegratorState.B) annotation(
    Line(points = {{-70, 82}, {-44, 82}}, color = {0, 0, 127}, thickness = 0.5));
  connect(augmentIntegratorState.Aaug, discreteLQR.A) annotation(
    Line(points = {{-20, 84}, {0, 84}, {0, 58}, {28, 58}}, color = {0, 0, 127}, thickness = 0.5));
  connect(augmentIntegratorState.Baug, discreteLQR.B) annotation(
    Line(points = {{-20, 74}, {-4, 74}, {-4, 54}, {28, 54}}, color = {0, 0, 127}, thickness = 0.5));
  connect(angularRatesOutput.C, augmentIntegratorState.C) annotation(
    Line(points = {{-71, 54}, {-60, 54}, {-60, 76}, {-44, 76}}, color = {0, 0, 127}, thickness = 0.5));
  connect(angularRatesOutput.bus, bus) annotation(
    Line(points = {{-92, 54}, {-100, 54}, {-100, 100}, {100, 100}, {100, 0}}, color = {255, 204, 51}, thickness = 0.5));
  connect(v_body.q_bw, bus.q_est) annotation(
    Line(points = {{-44, 138}, {100, 138}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.v_est, v_body.x_w) annotation(
    Line(points = {{100, 0}, {100, 126}, {-44, 126}}, thickness = 0.5));
  connect(v_body.x_b, x_1_6.v1) annotation(
    Line(points = {{-66, 132}, {-100, 132}, {-100, -16}, {-80, -16}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.w_est, x_1_6.v2) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, -24}, {-80, -24}}, thickness = 0.5));
  connect(x_1_6.vc, vectorConcatenate.v1) annotation(
    Line(points = {{-56, -20}, {-40, -20}, {-40, -28}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.control_position_meas[1:3], vectorConcatenate.v2) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, -36}, {-40, -36}}, thickness = 0.5));
  connect(vectorConcatenate.vc, vectorConcatenate2.v1) annotation(
    Line(points = {{-16, -32}, {-10, -32}, {-10, -42}, {2, -42}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorConcatenate1.vc, vectorConcatenate2.v2) annotation(
    Line(points = {{-42, -76}, {-24, -76}, {-24, -50}, {2, -50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorConcatenate2.vc, discreteLQR.x) annotation(
    Line(points = {{26, -46}, {36, -46}, {36, 8}, {14, 8}, {14, 42}, {28, 42}}, color = {0, 0, 127}, thickness = 0.5));
  connect(p_err_int, vectorConcatenate1.v1[1]) annotation(
    Line(points = {{-120, -100}, {-86, -100}, {-86, -72}, {-66, -72}}, color = {0, 0, 127}));
  connect(qr_err_int, vectorConcatenate1.v2) annotation(
    Line(points = {{-120, -60}, {-82, -60}, {-82, -80}, {-66, -80}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.velocity_guidace, discreteLQR.enable) annotation(
    Line(points = {{100, 0}, {100, 80}, {40, 80}, {40, 60}}, color = {255, 0, 255}));
  annotation(
    Icon(graphics = {Rectangle(fillColor = {255, 242, 254}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 12), Polygon(origin = {-72, -22}, fillColor = {91, 91, 91}, fillPattern = FillPattern.Solid, points = {{-2, -12}, {8, -22}, {22, 12}, {12, 22}, {-22, 8}, {-2, -12}}), Polygon(origin = {-41, 9}, fillColor = {155, 183, 193}, fillPattern = FillPattern.Solid, points = {{-43, -33}, {25, 35}, {45, 45}, {35, 25}, {-33, -43}, {-37, -39}, {-43, -33}}), Line(origin = {24.4844, 12.5364}, points = {{-56, -7}, {56, 7}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 12), Line(origin = {30.3844, 41.6364}, points = {{-15.9035, 23.9035}, {0.0964559, 17.9035}, {8.09646, 7.90354}, {14.0965, -4.09646}, {16.0965, -24.0965}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 21), Text(origin = {-1, -68}, extent = {{-77, 18}, {77, -18}}, textString = "vtrack"), Text(origin = {2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end AngularRateControl;
