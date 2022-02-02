within RocketControl.GNC.Control;

block BodyVelocityControlWithActuator
  parameter Real Qvec[9] = {0, 10, 10, 100, 0, 0, 0.1, 0.1, 0.1} * 100;
  parameter Real Rvec[3] = {1, 1, 0.4} * 20;
  RocketControl.GNC.Control.LinearStateMatrices.RocketAndActuator rocketAndActuator(CA0 = 0.4200, CA_a = -0.0277, CA_b = -0.0277, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22, wa = 13) annotation(
    Placement(visible = true, transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.ContinuousLQR continuousLQR(m = 3, n = 9, useEnablePort = true)  annotation(
    Placement(visible = true, transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Matrices.MatrixConstant Q(n = 9, val = diagonal(Qvec))  annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Matrices.MatrixConstant R(n = 3, val = diagonal(Rvec))  annotation(
    Placement(visible = true, transformation(origin = {-50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.Control.Control2Deflection control2Deflection annotation(
    Placement(visible = true, transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConcatenate x_lq(n2 = 3)  annotation(
    Placement(visible = true, transformation(origin = {-50, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConstant vectorConstant1(k = {0, 0, 0, 0}, n = 4)  annotation(
    Placement(visible = true, transformation(origin = {40, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConcatenate vectorConcatenate(n1 = 6, n2 = 3) annotation(
    Placement(visible = true, transformation(origin = {-14, -92}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Actuators.TFServoMotor servoMotor(a = {0.07692, 1}, b = {0, 1}, nservos = 3) annotation(
    Placement(visible = true, transformation(origin = {-14, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  RocketControl.Blocks.Math.Vectors.VectorConstant vectorConstant2(k = {0, 1, 0}, n = 3) annotation(
    Placement(visible = true, transformation(origin = {-124, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(bus.control_enable, continuousLQR.enable) annotation(
    Line(points = {{100, 0}, {100, 100}, {10, 100}, {10, 10}}, color = {255, 0, 255}));
  connect(bus, rocketAndActuator.bus) annotation(
    Line(points = {{100, 0}, {100, 100}, {-100, 100}, {-100, 50}, {-60, 50}}, thickness = 0.5));
  connect(rocketAndActuator.A, continuousLQR.A) annotation(
    Line(points = {{-38, 56}, {-16, 56}, {-16, 8}, {-2, 8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(rocketAndActuator.B, continuousLQR.B) annotation(
    Line(points = {{-38, 46}, {-22, 46}, {-22, 4}, {-2, 4}}, color = {0, 0, 127}, thickness = 0.5));
  connect(Q.k, continuousLQR.Q) annotation(
    Line(points = {{-39, 0}, {-2, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(R.k, continuousLQR.R) annotation(
    Line(points = {{-39, -30}, {-22, -30}, {-22, -4}, {-2, -4}}, color = {0, 0, 127}, thickness = 0.5));
  connect(x_lq.vc, vectorConcatenate.v1) annotation(
    Line(points = {{-38, -60}, {-36, -60}, {-36, -88}, {-26, -88}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorConcatenate.vc, continuousLQR.x) annotation(
    Line(points = {{-2, -92}, {10, -92}, {10, -12}, {-10, -12}, {-10, -8}, {-2, -8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(servoMotor.setpoint, continuousLQR.u) annotation(
    Line(points = {{-2, 78}, {-2, 40}, {22, 40}, {22, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(servoMotor.servo_pos_nonsat, vectorConcatenate.v2) annotation(
    Line(points = {{-25, 71}, {-76, 71}, {-76, -96}, {-26, -96}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorConstant2.v, x_lq.v1) annotation(
    Line(points = {{-112, -40}, {-62, -40}, {-62, -56}}, color = {0, 0, 127}, thickness = 0.5));
  connect(servoMotor.servo_pos_nonsat, control2Deflection.u) annotation(
    Line(points = {{-25, 71}, {48, 71}, {48, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(control2Deflection.deflection, bus.fin_setpoint) annotation(
    Line(points = {{72, 0}, {100, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bus.w_est, x_lq.v2) annotation(
    Line(points = {{100, 0}, {100, 100}, {-102, 100}, {-102, -64}, {-62, -64}}, thickness = 0.5));
  annotation(
    Icon(graphics = {Rectangle(fillColor = {255, 242, 254}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 12), Polygon(origin = {-72, -22}, fillColor = {91, 91, 91}, fillPattern = FillPattern.Solid, points = {{-2, -12}, {8, -22}, {22, 12}, {12, 22}, {-22, 8}, {-2, -12}}), Polygon(origin = {-41, 9}, fillColor = {155, 183, 193}, fillPattern = FillPattern.Solid, points = {{-43, -33}, {25, 35}, {45, 45}, {35, 25}, {-33, -43}, {-37, -39}, {-43, -33}}), Line(origin = {24.4844, 12.5364}, points = {{-56, -7}, {56, 7}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 12), Line(origin = {30.3844, 41.6364}, points = {{-15.9035, 23.9035}, {0.0964559, 17.9035}, {8.09646, 7.90354}, {14.0965, -4.09646}, {16.0965, -24.0965}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 21), Text(origin = {-1, -68}, extent = {{-77, 18}, {77, -18}}, textString = "vtrack"), Text(origin = {2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end BodyVelocityControlWithActuator;
