within RocketControl.Blocks.Math.Vector;

model VectorConcatenate
  extends Internal.VectorIcon;
  parameter Integer n1(min = 1) = 3 annotation(
    Evaluate = true);
  parameter Integer n2(min = 1) = 3 annotation(
    Evaluate = true);
  Modelica.Blocks.Interfaces.RealOutput vc[n1 + n2] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput v2[n2] annotation(
    Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput v1[n1] annotation(
    Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  vc = cat(1, v1, v2);
  annotation(
    Icon(graphics = {Text(origin = {0, -250}, textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {1, 5}, extent = {{-85, 59}, {85, -59}}, textString = "{v1,v2}")}));
end VectorConcatenate;
