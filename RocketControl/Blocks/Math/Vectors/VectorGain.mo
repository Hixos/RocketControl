within RocketControl.Blocks.Math.Vectors;

block VectorGain
 extends RocketControl.Icons.VectorBlock;
  parameter Integer n(min = 1) = 3 annotation(
    Evaluate = true);
  parameter Boolean external_gain = true;
  parameter Real gain;
  Modelica.Blocks.Interfaces.RealInput v[n] annotation(
    Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput k annotation(
    Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput vk[n] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  if not external_gain then
    k = gain;
  end if;
  vk = k * v;
  annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {-130, 20}, lineColor = {118, 118, 118}, fillColor = {118, 118, 118}, extent = {{-30, 20}, {30, -20}}, textString = "v"), Text(origin = {-130, -100}, lineColor = {118, 118, 118}, extent = {{-30, 20}, {30, -20}}, textString = "k"), Text(extent = {{-100, 100}, {100, -100}}, textString = "kv")}));
end VectorGain;
