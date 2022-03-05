within RocketControl.GNC.Control;

block AccelerationRollRateControlVel
  outer World.SimOptions opt;
  parameter Real Qvec[:] = {0, 1, 1, 0, 0, 0, 10, 10} * 0.01;
  parameter Real Rvec[:] = {1, 1, 0.4} * 300;
  RocketControl.Blocks.Math.Matrices.MatrixConstant Q(n = size(Qvec, 1), val = diagonal(Qvec))  annotation(
    Placement(visible = true, transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Matrices.MatrixConstant R(n = 3, val = diagonal(Rvec))  annotation(
    Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
    Placement(visible = true, transformation(origin = {70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput acc_err_int[3] annotation(
    Placement(visible = true, transformation(origin = {-116, -88}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConcatenate x_lq(n2 = 3)  annotation(
    Placement(visible = true, transformation(origin = {-50, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConstant vectorConstant1(k = {0, 0, 0, 0}, n = 4)  annotation(
    Placement(visible = true, transformation(origin = {40, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.DiscreteLQR discreteLQR(dt = opt.samplePeriodMs / 1000, m = 3, n = 8, s = 1, useEnablePort = true)  annotation(
    Placement(visible = true, transformation(origin = {40, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConcatenate vectorConcatenate(n1 = 6, n2 = 2) annotation(
    Placement(visible = true, transformation(origin = {-18, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AugmentIntegratorState augmentIntegratorState(m = 3, n = 6, p = 2)  annotation(
    Placement(visible = true, transformation(origin = {-32, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.LinearStateMatrices.RocketOnly rocketOnly annotation(
    Placement(visible = true, transformation(origin = {-78, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.LinearOutputMatrices.AccelerationVelOutput accelerationVelOutput annotation(
    Placement(visible = true, transformation(origin = {-78, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(Q.k, discreteLQR.Q) annotation(
    Line(points = {{-39, 40}, {-20, 40}, {-20, 50}, {28, 50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(R.k, discreteLQR.R) annotation(
    Line(points = {{-39, 10}, {-16, 10}, {-16, 46}, {28, 46}}, color = {0, 0, 127}, thickness = 0.5));
  connect(discreteLQR.u, control2Deflection.u[1:3]) annotation(
    Line(points = {{51, 50}, {58, 50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(control2Deflection.deflection, bus.fin_setpoint) annotation(
    Line(points = {{81, 50}, {83.5, 50}, {83.5, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(x_lq.vc, vectorConcatenate.v1) annotation(
    Line(points = {{-39, -16}, {-30.5, -16}, {-30.5, -26}, {-30, -26}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorConcatenate.vc, discreteLQR.x) annotation(
    Line(points = {{-7, -30}, {12, -30}, {12, 24}, {-12, 24}, {-12, 42}, {28, 42}}, color = {0, 0, 127}, thickness = 0.5));
  connect(augmentIntegratorState.Aaug, discreteLQR.A) annotation(
    Line(points = {{-20, 84}, {0, 84}, {0, 58}, {28, 58}}, color = {0, 0, 127}, thickness = 0.5));
  connect(augmentIntegratorState.Baug, discreteLQR.B) annotation(
    Line(points = {{-20, 74}, {-4, 74}, {-4, 54}, {28, 54}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.velocity_guidace, discreteLQR.enable) annotation(
    Line(points = {{100, 0}, {100, 102}, {40, 102}, {40, 60}}, color = {255, 0, 255}));
  connect(bus, rocketOnly.bus) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 86}, {-88, 86}}, thickness = 0.5));
  connect(rocketOnly.A, augmentIntegratorState.A) annotation(
    Line(points = {{-66, 92}, {-58, 92}, {-58, 86}, {-44, 86}}, color = {0, 0, 127}, thickness = 0.5));
  connect(rocketOnly.B, augmentIntegratorState.B) annotation(
    Line(points = {{-66, 82}, {-44, 82}}, color = {0, 0, 127}, thickness = 0.5));
  connect(accelerationVelOutput.C, augmentIntegratorState.C) annotation(
    Line(points = {{-66, 56}, {-58, 56}, {-58, 76}, {-44, 76}}, color = {0, 0, 127}, thickness = 0.5));
  connect(acc_err_int[2], vectorConcatenate.v2[2]) annotation(
    Line(points = {{-116, -88}, {-50, -88}, {-50, -34}, {-30, -34}}, color = {0, 0, 127}, thickness = 0.5));
  connect(acc_err_int[3], vectorConcatenate.v2[3]) annotation(
    Line(points = {{-116, -88}, {-50, -88}, {-50, -34}, {-30, -34}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.a_meas, x_lq.v1) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, -12}, {-62, -12}}, thickness = 0.5));
  connect(bus.w_dot_meas, x_lq.v2) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, -20}, {-62, -20}}, thickness = 0.5));
  annotation(
    Icon(graphics = {Rectangle(fillColor = {255, 242, 254}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 12), Polygon(origin = {-72, -22}, fillColor = {91, 91, 91}, fillPattern = FillPattern.Solid, points = {{-2, -12}, {8, -22}, {22, 12}, {12, 22}, {-22, 8}, {-2, -12}}), Polygon(origin = {-41, 9}, fillColor = {155, 183, 193}, fillPattern = FillPattern.Solid, points = {{-43, -33}, {25, 35}, {45, 45}, {35, 25}, {-33, -43}, {-37, -39}, {-43, -33}}), Line(origin = {24.4844, 12.5364}, points = {{-56, -7}, {56, 7}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 12), Line(origin = {30.3844, 41.6364}, points = {{-15.9035, 23.9035}, {0.0964559, 17.9035}, {8.09646, 7.90354}, {14.0965, -4.09646}, {16.0965, -24.0965}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 21), Text(origin = {-1, -68}, extent = {{-77, 18}, {77, -18}}, textString = "vtrack"), Text(origin = {2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end AccelerationRollRateControlVel;
