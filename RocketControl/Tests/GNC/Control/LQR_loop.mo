within RocketControl.Tests.GNC.Control;

model LQR_loop
  RocketControl.GNC.ContinuousLQR continuousLQR(g = -0.5, m = 1, n = 2, useEnablePort = false)  annotation(
    Placement(visible = true, transformation(origin = {16, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Matrices.MatrixConstant A(n = 2, val = [2, 1; 0, -1])  annotation(
    Placement(visible = true, transformation(origin = {-64, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Matrices.MatrixConstant B(m = 1, n = 2, val = [0; 1])  annotation(
    Placement(visible = true, transformation(origin = {-64, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Matrices.MatrixConstant Q(m = 2, n = 2, val = diagonal({1, 1})) annotation(
    Placement(visible = true, transformation(origin = {-66, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Matrices.MatrixConstant R(m = 1, n = 1, val = [1]) annotation(
    Placement(visible = true, transformation(origin = {-66, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 
  Modelica.Blocks.Continuous.TransferFunction transferFunction(a = {1, 1}, b = {1}, initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = 2)  annotation(
    Placement(visible = true, transformation(origin = {64, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.Blocks.Sources.BooleanExpression booleanExpression(y = time > 0.5 and time < 9)  annotation(
    Placement(visible = true, transformation(origin = {-36, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
    
    Real x1;
equation
  der(x1) = 2*x1 + 1*transferFunction.y;
  continuousLQR.x[1] = x1;
  connect(A.k, continuousLQR.A) annotation(
    Line(points = {{-52, 26}, {-14, 26}, {-14, 8}, {4, 8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(B.k, continuousLQR.B) annotation(
    Line(points = {{-52, -4}, {-24, -4}, {-24, 4}, {4, 4}}, color = {0, 0, 127}, thickness = 0.5));
  connect(Q.k, continuousLQR.Q) annotation(
    Line(points = {{-54, -30}, {-14, -30}, {-14, 0}, {4, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(R.k, continuousLQR.R) annotation(
    Line(points = {{-54, -60}, {-6, -60}, {-6, -4}, {4, -4}}, color = {0, 0, 127}, thickness = 0.5));
  connect(continuousLQR.u[1], transferFunction.u) annotation(
    Line(points = {{28, 0}, {52, 0}}, color = {0, 0, 127}));
  connect(transferFunction.y, continuousLQR.x[2]) annotation(
    Line(points = {{75, 0}, {78, 0}, {78, -54}, {-2, -54}, {-2, -8}, {4, -8}}, color = {0, 0, 127}));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end LQR_loop;
