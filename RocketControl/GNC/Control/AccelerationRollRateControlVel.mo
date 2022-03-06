within RocketControl.GNC.Control;

block AccelerationRollRateControlVel
  outer World.SimOptions opt;
  parameter Real Qvec[:] = {0, 0, 0, 0, 0, 0, 0, 0,0,10,10} * 0.01;
  parameter Real Rvec[:] = {1, 1, 1} * 300;
  RocketControl.Blocks.Math.Matrices.MatrixConstant Q(n = size(Qvec, 1), val = diagonal(Qvec))  annotation(
    Placement(visible = true, transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Matrices.MatrixConstant R(n = 3, val = diagonal(Rvec))  annotation(
    Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
    Placement(visible = true, transformation(origin = {70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput acc_err_int[3] annotation(
    Placement(visible = true, transformation(origin = {-116, -88}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConcatenate x_lq(n2 = 3)  annotation(
    Placement(visible = true, transformation(origin = {-52, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConstant vectorConstant1(k = {0, 0, 0, 0}, n = 4)  annotation(
    Placement(visible = true, transformation(origin = {82, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.DiscreteLQR discreteLQR(dt = opt.samplePeriodMs / 1000, m = 3, n = 9, s = 1, useEnablePort = true)  annotation(
    Placement(visible = true, transformation(origin = {40, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConcatenate vectorConcatenate(n1 = 6, n2 = 3) annotation(
    Placement(visible = true, transformation(origin = {8, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.LinearOutputMatrices.AccelerationVelOutput accelerationVelOutput annotation(
    Placement(visible = true, transformation(origin = {-78, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  LinearStateMatrices.RocketAndActuatorVel rocket(CA0 = 0.4200, CA_a = -0.1725, CA_b = -0.1725, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLL_p = -19, CLM_a = -37.2959, CLM_dp = 21.8445, CLM_q = -1813, CLN_b = 37.2959, CLN_dy = 21.8445, CLN_r = -1813, CN_a = 24.0744, CN_dp = 3.4045, CN_q = 154, CY_b = -24.0744, CY_dy = 3.4045, CY_r = 154, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22, wa = 13) annotation(
    Placement(visible = true, transformation(origin = {-82, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorGain vectorGain(external_gain = false, gain = 1)  annotation(
    Placement(visible = true, transformation(origin = {-144, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorGain vectorGain1(external_gain = false, gain = 1) annotation(
    Placement(visible = true, transformation(origin = {-146, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(Q.k, discreteLQR.Q) annotation(
    Line(points = {{-39, 40}, {-20, 40}, {-20, 50}, {28, 50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(R.k, discreteLQR.R) annotation(
    Line(points = {{-39, 10}, {-16, 10}, {-16, 46}, {28, 46}}, color = {0, 0, 127}, thickness = 0.5));
  connect(discreteLQR.u, control2Deflection.u[1:3]) annotation(
    Line(points = {{51, 50}, {58, 50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(control2Deflection.deflection, bus.fin_setpoint) annotation(
    Line(points = {{81, 50}, {83.5, 50}, {83.5, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.velocity_guidace, discreteLQR.enable) annotation(
    Line(points = {{100, 0}, {100, 102}, {40, 102}, {40, 60}}, color = {255, 0, 255}));
  connect(rocket.bus, bus) annotation(
    Line(points = {{-92, 86}, {-100, 86}, {-100, 100}, {100, 100}, {100, 0}}, color = {255, 204, 51}, thickness = 0.5));
  connect(x_lq.vc, vectorConcatenate.v1) annotation(
    Line(points = {{-41, -16}, {-32, -16}, {-32, -34}, {-4, -34}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.cpos_dot[1:3], vectorConcatenate.v2) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, -42}, {-4, -42}}, thickness = 0.5));
  connect(rocket.A, discreteLQR.A) annotation(
    Line(points = {{-70, 92}, {28, 92}, {28, 58}}, color = {0, 0, 127}, thickness = 0.5));
  connect(rocket.B, discreteLQR.B) annotation(
    Line(points = {{-70, 82}, {28, 82}, {28, 54}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorConcatenate.vc, discreteLQR.x) annotation(
    Line(points = {{19, -38}, {28, -38}, {28, 42}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorGain.vk, x_lq.v1) annotation(
    Line(points = {{-132, 2}, {-64, 2}, {-64, -12}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorGain1.vk, x_lq.v2) annotation(
    Line(points = {{-134, -24}, {-64, -24}, {-64, -20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.a_meas, vectorGain.v) annotation(
    Line(points = {{100, 0}, {100, 100}, {-180, 100}, {-180, 8}, {-156, 8}}, thickness = 0.5));
  connect(bus.w_dot, vectorGain1.v) annotation(
    Line(points = {{100, 0}, {98, 0}, {98, 100}, {-180, 100}, {-180, -18}, {-158, -18}}, thickness = 0.5));
  annotation(
    Icon(graphics = {Rectangle(fillColor = {255, 242, 254}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 12), Polygon(origin = {-72, -22}, fillColor = {91, 91, 91}, fillPattern = FillPattern.Solid, points = {{-2, -12}, {8, -22}, {22, 12}, {12, 22}, {-22, 8}, {-2, -12}}), Polygon(origin = {-41, 9}, fillColor = {155, 183, 193}, fillPattern = FillPattern.Solid, points = {{-43, -33}, {25, 35}, {45, 45}, {35, 25}, {-33, -43}, {-37, -39}, {-43, -33}}), Line(origin = {24.4844, 12.5364}, points = {{-56, -7}, {56, 7}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 12), Line(origin = {30.3844, 41.6364}, points = {{-15.9035, 23.9035}, {0.0964559, 17.9035}, {8.09646, 7.90354}, {14.0965, -4.09646}, {16.0965, -24.0965}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 21), Text(origin = {-1, -68}, extent = {{-77, 18}, {77, -18}}, textString = "vtrack"), Text(origin = {2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end AccelerationRollRateControlVel;
