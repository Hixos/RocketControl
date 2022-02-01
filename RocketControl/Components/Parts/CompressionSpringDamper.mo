within RocketControl.Components.Parts;

model CompressionSpringDamper "Spring and damper in parallel that act only on a specific direction and can be enabled / disabled"
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
  parameter Real n[3] = {1, 0, 0} "Direction along which the spring is acting";
  parameter SI.ModulusOfElasticity c;
  parameter SI.DampingCoefficient d;
  Components.Forces.CompressionSpring linearSpring(c = c, n = n) annotation(
    Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Forces.CompressionDamper linearDamper(d = d, n = n) annotation(
    Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(linearSpring.frame_b, frame_b) annotation(
    Line(points = {{10, 40}, {60, 40}, {60, 0}, {100, 0}}));
  connect(linearDamper.frame_b, frame_b) annotation(
    Line(points = {{10, -40}, {60, -40}, {60, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(frame_a, linearDamper.frame_a) annotation(
    Line(points = {{-100, 0}, {-60, 0}, {-60, -40}, {-10, -40}}));
  connect(linearSpring.frame_a, frame_a) annotation(
    Line(points = {{-10, 40}, {-60, 40}, {-60, 0}, {-100, 0}}, color = {95, 95, 95}));
  annotation(
    Icon(graphics = {Line(points = {{-52, -40}, {68, -40}}), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{70, 30}, {130, -30}}, endAngle = 360), Text(origin = {-82, 122}, extent = {{-150, -75}, {150, -45}}, textString = "d=%d"), Line(points = {{38, -70}, {80, -70}}), Text(lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{-70, 30}, {-130, -30}}, endAngle = 360), Line(points = {{80, 40}, {80, -70}}), Text(visible = false, lineColor = {255, 0, 0}, extent = {{-62, 50}, {-140, 30}}, textString = "R=0"), Text(origin = {-82, 178}, extent = {{-150, -75}, {150, -45}}, textString = "n=%n"), Text(visible = false, lineColor = {255, 0, 0}, extent = {{62, 50}, {140, 30}}, textString = "R=0"), Text(visible = false, origin = {0, -28}, lineColor = {255, 0, 0}, extent = {{62, 50}, {140, 30}}, textString = "R=0"), Rectangle(fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-52, -40}, {38, -100}}), Line(points = {{-80, 40}, {-60, 40}, {-45, 10}, {-15, 70}, {15, 10}, {45, 70}, {60, 40}, {80, 40}}), Text(origin = {-82, 154}, extent = {{-150, -75}, {150, -45}}, textString = "c=%c"), Line(points = {{-80, -70}, {-52, -70}}), Text(visible = false, lineColor = {255, 0, 0}, extent = {{-62, 50}, {-140, 30}}, textString = "R=0"), Text(lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{70, 30}, {130, -30}}, endAngle = 360), Line(points = {{80, 0}, {100, 0}}), Line(points = {{-52, -100}, {68, -100}}), Line(visible = false, points = {{-100, -101}, {-100, -80}, {-6, -80}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{-70, 30}, {-130, -30}}, endAngle = 360), Line(origin = {-80, -15}, points = {{0, 55}, {0, -55}}), Line(origin = {-89, 0}, points = {{9, 0}, {-7, 0}, {-9, 0}}), Line(origin = {-24.6905, -11.2486}, points = {{44, 0}, {4, 0}}, thickness = 0.75), Polygon(origin = {-30, -12}, rotation = 180, fillPattern = FillPattern.Solid, points = {{10, 20}, {-10, 0}, {10, -20}, {0, 0}, {10, 20}}), Polygon(origin = {30, -12}, fillPattern = FillPattern.Solid, points = {{10, 20}, {-10, 0}, {10, -20}, {0, 0}, {10, 20}})}));
end CompressionSpringDamper;
