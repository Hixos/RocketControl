within RocketControl.Components.Actuators;

block TFServoMotor
parameter Integer nservos(min = 1) = 4;
  parameter Real b[:] = {1} "Numerator coefficients of transfer function (e.g., 2*s+3 is specified as {2,3})";
  parameter Real a[:] = {1} "Denominator coefficients of transfer function (e.g., 5*s+6 is specified as {5,6})";
  parameter SI.Angle saturation_angle = from_deg(10);
  Modelica.Blocks.Interfaces.RealInput setpoint[nservos](each final unit = "rad", each final quantity="Angle", each displayUnit="deg") annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput servo_pos[nservos](each final unit = "rad", each final quantity="Angle", each displayUnit="deg") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.TransferFunction servoResponse[nservos](each a = a, each b = b, each initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(
    Placement(visible = true, transformation(origin = {-24, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.Limiter servoSaturation[nservos](each uMax = saturation_angle)  annotation(
    Placement(visible = true, transformation(origin = {32, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput servo_pos_nonsat[nservos](each final unit = "rad", each final quantity="Angle", each displayUnit="deg") annotation(
    Placement(visible = true, transformation(origin = {110, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Clocked.VectorHold vectorHold(n = nservos) annotation(
    Placement(visible = true, transformation(origin = {-68, -6.66134e-16}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
equation
  connect(servoResponse.y, servoSaturation.u) annotation(
    Line(points = {{-13, 0}, {20, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(servoSaturation.y, servo_pos) annotation(
    Line(points = {{44, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(servoResponse.y, servo_pos_nonsat) annotation(
    Line(points = {{-13, 0}, {-4, 0}, {-4, 58}, {110, 58}}, color = {0, 0, 127}, thickness = 0.5));
  connect(setpoint, vectorHold.u) annotation(
    Line(points = {{-120, 0}, {-76, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vectorHold.y, servoResponse.u) annotation(
    Line(points = {{-62, 0}, {-36, 0}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(graphics = {Rectangle(fillColor = {238, 238, 238}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Polygon(origin = {45, 0}, fillColor = {93, 255, 147}, fillPattern = FillPattern.Forward, points = {{-35, 62}, {-35, -62}, {35, -36}, {35, 20}, {-35, 62}}), Polygon(origin = {-28, 0}, fillColor = {203, 203, 203}, fillPattern = FillPattern.HorizontalCylinder, points = {{38, 8}, {0, 8}, {0, 40}, {-38, 40}, {-38, -40}, {0, -40}, {0, -8}, {38, -8}, {38, 8}}), Line(origin = {-7.28, -1.55}, points = {{8.7929, 18}, {4.79289, 30}, {-5.20711, 30}, {-11.2071, 18}, {-11.2071, -4}, {-11.2071, -18}, {-5.20711, -30}, {4.79289, -30}, {8.79289, -18}, {8.79289, -18}}, color = {255, 0, 0}, arrow = {Arrow.Filled, Arrow.None}, arrowSize = 10), Text(origin = {0, -125}, lineColor = {0, 0, 255}, extent = {{-140, 25}, {140, -25}}, textString = "%name")}));
end TFServoMotor;
