within RocketControl.Components.Parts;

model ParachuteLine2
  import Modelica.Mechanics.MultiBody.Types;
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialLineForce;
  parameter SI.TranslationalSpringConstant c(final min=0) "Spring constant";
  parameter SI.Length s_unstretched=0 "Unstretched spring length";
  parameter SI.TranslationalDampingConstant d(final min=0) = 0
    "Damping constant";

  Modelica.Blocks.Interfaces.BooleanOutput extended annotation(
    Placement(visible = true, transformation(origin = {106, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
protected
  SI.Force f_d "Damping force";
initial equation
extended = false;
equation
  when s > s_unstretched then
    extended =  true;
  end when;
  
  if s > s_unstretched then
    f_d = d*der(s);
    f = c*(s - s_unstretched) + f_d;
  else
    f_d = 0;
    f = 0;
  end if;
  
  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(fillColor = {255, 255, 196}, fillPattern = FillPattern.Solid, extent = {{-100, 40}, {100, -40}}), Text(lineColor = {128, 128, 128}, extent = {{100, -25}, {136, -50}}, textString = "b"), Text(origin = {-10, 0}, extent = {{-150, 105}, {150, 135}}, textString = "c=%c"), Text(origin = {-4, 2}, extent = {{-150, 70}, {150, 100}}, textString = "d=%d"), Text(visible = false, lineColor = {255, 0, 0}, extent = {{62, 50}, {140, 30}}, textString = "R=0"), Text(lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Text(visible = false, lineColor = {255, 0, 0}, extent = {{62, 50}, {140, 30}}, textString = "R=0"), Line(origin = {2.2149, -0.0861658}, points = {{-98, -0.0336411}, {-42, -0.0336411}, {-32, -20.0336}, {-12, 19.9664}, {8, -20.0336}, {28, 19.9664}, {40, -0.0336411}, {94, -0.0336411}}), Text(origin = {10, -70}, lineColor = {170, 0, 255}, fillColor = {255, 0, 255}, extent = {{-70, 28}, {70, -28}}, textString = "s > %s_unstretched")}));
end ParachuteLine2;
