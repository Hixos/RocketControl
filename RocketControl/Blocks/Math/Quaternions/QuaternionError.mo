within RocketControl.Blocks.Math.Quaternions;

model QuaternionError
  extends RocketControl.Icons.QuaternionBlock;
  import RocketControl.Math.Quaternions.*;
  Modelica.Blocks.Interfaces.RealInput q1[4] annotation(
    Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 42}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput q2[4] annotation(
    Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput qerr[4] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  qerr = quatmolt(q1, quatinv(q2));
  annotation(
    Icon(graphics = {Text(origin = {-2, 8}, extent = {{-88, 64}, {88, -64}}, textString = "θerr")}));
end QuaternionError;
