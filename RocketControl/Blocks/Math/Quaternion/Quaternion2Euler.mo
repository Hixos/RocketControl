within RocketControl.Blocks.Math.Quaternion;

model Quaternion2Euler "Converts a quaternion to the equivalent ZYX euler angle representation"
  extends Internal.QuaternionIcon;
  Modelica.Blocks.Interfaces.RealOutput eul[3](each final quantity = "Angle", each final unit = "rad", each displayUnit = "deg") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput q[4] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  eul = quat2euler(q);
  annotation(
    Icon(graphics = {Text(origin = {7, 75}, extent = {{-93, 25}, {93, -25}}, textString = "quat", horizontalAlignment = TextAlignment.Left), Text(origin = {-4, -69}, extent = {{-96, 25}, {96, -25}}, textString = "y,p,r", horizontalAlignment = TextAlignment.Right), Line(origin = {-5, 0}, points = {{-75, 0}, {83, 0}}, thickness = 0.5, arrow = {Arrow.None, Arrow.Open}, arrowSize = 15)}));
end Quaternion2Euler;
