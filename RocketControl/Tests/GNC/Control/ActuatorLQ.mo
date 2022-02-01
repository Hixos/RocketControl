within RocketControl.Tests.GNC.Control;

model ActuatorLQ
  RocketControl.GNC.Control.LinearStateMatrices.RocketAndActuator systemMatrices(CA0 = 0.4200, CA_a = -0.0277, CA_b = -0.0277, CA_dp = 0.4001, CA_dr = 0.5739, CA_ds = 0.8899, CA_dy = 0.4001, CLL_dr = 2.3963, CLM_a = -37.2959, CLM_dp = 21.8445, CLN_b = 37.2959, CLN_dy = 21.8445, CN_a = 24.0744, CN_dp = 3.4045, CY_b = -24.0744, CY_dy = 3.4045, Is = 6.437, Ix = 0.06, S = pi * 0.15 ^ 2 / 4, c = 0.15, m = 22, wa = 13) annotation(
    Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.GNC.ContinuousLQR continuousLQR(g = -1, m = 3, maxiter = 15, n = 9, tol = 1e-40, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Matrices.MatrixConstant R(n = 3, val = diagonal({1, 1, 0.4} * 20)) annotation(
    Placement(visible = true, transformation(origin = {-30, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Matrices.MatrixConstant Q(n = 9, val = diagonal({0, 10, 10, 100, 0, 0, 0, 0, 0})) annotation(
    Placement(visible = true, transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Blocks.Sources.BooleanConstant booleanConstant annotation(
    Placement(visible = true, transformation(origin = {-10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
RocketControl.Math.Blocks.Vector.VectorConstant vel(k = {200, 0, 0})  annotation(
    Placement(visible = true, transformation(origin = {-90, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
RocketControl.Math.Blocks.Vector.VectorConstant ang_vel(k = {0, 0, 0})  annotation(
    Placement(visible = true, transformation(origin = {-90, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
RocketControl.Math.Blocks.Vector.VectorConstant x_est(k = {0, 0, -1000})  annotation(
    Placement(visible = true, transformation(origin = {-92, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
RocketControl.Math.Blocks.Vector.VectorConstant control(k = {1, 0, 0})  annotation(
    Placement(visible = true, transformation(origin = {-92, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate annotation(
    Placement(visible = true, transformation(origin = {-48, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
RocketControl.Math.Blocks.Vector.VectorConcatenate vectorConcatenate1(n1 = 6)  annotation(
    Placement(visible = true, transformation(origin = {2, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant1(k = {10, 0}, n = 2) annotation(
    Placement(visible = true, transformation(origin = {-116, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Blocks.Continuous.TransferFunction transferFunction(a = {0.07692, 1}, b = {0, 1}, initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(
    Placement(visible = true, transformation(origin = {85, -33}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
RocketControl.Math.Blocks.Vector.VectorConstant vectorConstant(k = {0, 0}, n = 2) annotation(
    Placement(visible = true, transformation(origin = {-92, -116}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(systemMatrices.A, continuousLQR.A) annotation(
    Line(points = {{-19, 35}, {10, 35}, {10, -2}, {18, -2}}, color = {0, 0, 127}, thickness = 0.5));
  connect(R.k, continuousLQR.R) annotation(
    Line(points = {{-19, -42}, {6, -42}, {6, -14}, {18, -14}}, color = {0, 0, 127}, thickness = 0.5));
  connect(Q.k, continuousLQR.Q) annotation(
    Line(points = {{-19, -10}, {18, -10}}, color = {0, 0, 127}, thickness = 0.5));
  connect(systemMatrices.B, continuousLQR.B) annotation(
    Line(points = {{-19, 25}, {6, 25}, {6, -6}, {18, -6}}, color = {0, 0, 127}, thickness = 0.5));
  connect(booleanConstant.y, continuousLQR.enable) annotation(
    Line(points = {{2, 90}, {30, 90}, {30, 0}}, color = {255, 0, 255}));
  connect(vel.v, systemMatrices.vel) annotation(
    Line(points = {{-79, 70}, {-58, 70}, {-58, 40}, {-42, 40}}, color = {0, 0, 127}, thickness = 0.5));
  connect(ang_vel.v, systemMatrices.ang_vel) annotation(
    Line(points = {{-78, 34}, {-42, 34}}, color = {0, 0, 127}, thickness = 0.5));
  connect(x_est.v, systemMatrices.x_est) annotation(
    Line(points = {{-80, 4}, {-62, 4}, {-62, 28}, {-42, 28}}, color = {0, 0, 127}, thickness = 0.5));
  connect(control.v, systemMatrices.control_cmd) annotation(
    Line(points = {{-80, -28}, {-56, -28}, {-56, 22}, {-42, 22}}, color = {0, 0, 127}, thickness = 0.5));
  connect(ang_vel.v, vectorConcatenate.v2) annotation(
    Line(points = {{-78, 34}, {-72, 34}, {-72, -78}, {-60, -78}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorConcatenate.vc, vectorConcatenate1.v1) annotation(
    Line(points = {{-36, -74}, {-10, -74}, {-10, -76}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorConcatenate1.vc, continuousLQR.x) annotation(
    Line(points = {{14, -80}, {40, -80}, {40, -44}, {12, -44}, {12, -18}, {18, -18}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vel.v[1], vectorConcatenate.v1[1]) annotation(
    Line(points = {{-78, 70}, {-72, 70}, {-72, -70}, {-60, -70}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorConstant1.v[1], vectorConcatenate.v1[2]) annotation(
    Line(points = {{-104, -64}, {-64, -64}, {-64, -70}, {-60, -70}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorConstant1.v[2], vectorConcatenate.v1[3]) annotation(
    Line(points = {{-104, -64}, {-60, -64}, {-60, -70}}, color = {0, 0, 127}, thickness = 0.5));
connect(continuousLQR.u[1], transferFunction.u) annotation(
    Line(points = {{42, -10}, {62, -10}, {62, -32}, {80, -32}}, color = {0, 0, 127}));
connect(transferFunction.y, vectorConcatenate1.v2[1]) annotation(
    Line(points = {{90, -32}, {92, -32}, {92, -98}, {-24, -98}, {-24, -84}, {-10, -84}}, color = {0, 0, 127}));
connect(vectorConstant.v[2], vectorConcatenate1.v2[2]) annotation(
    Line(points = {{-80, -116}, {-10, -116}, {-10, -84}}, color = {0, 0, 127}, thickness = 0.5));
connect(vectorConstant.v[1], vectorConcatenate1.v2[3]) annotation(
    Line(points = {{-80, -116}, {-10, -116}, {-10, -84}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end ActuatorLQ;
