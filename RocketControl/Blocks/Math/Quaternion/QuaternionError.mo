within RocketControl.Blocks.Math.Quaternion;

model QuaternionError
  extends Internal.QuaternionIcon;
  import RocketControl.Math.*;
  Modelica.Blocks.Interfaces.RealInput q1[4] annotation(
    Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 42}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput q2[4] annotation(
    Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput qerr[4] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  qerr = quatmolt(q1, quatinv(q2));
  annotation(
    Icon(graphics = {Text(origin = {2, 12}, extent = {{-86, 60}, {86, -60}}, textString = "q q*"), Ellipse(origin = {-24, -4}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Text(origin = {-28, -41}, extent = {{-16, 17}, {16, -17}}, textString = "1"), Text(origin = {48, -43}, extent = {{-16, 17}, {16, -17}}, textString = "2")}));
end QuaternionError;
