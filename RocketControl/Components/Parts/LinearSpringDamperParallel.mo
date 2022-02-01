within RocketControl.Components.Parts;

model LinearSpringDamperParallel "Spring and damper in parallel that act only on a specific direction and can be enabled / disabled"
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
  extends Interfaces.PartialConditionalEnablePort;
  parameter Real n[3] = {1, 0, 0} "Direction along which the spring is acting";
  parameter SI.Length s_0 = 0 "Undeformed spring length";
  parameter SI.Length s_start = 0 "Inital deformation";
  parameter Boolean s_fixed = false;
  parameter SI.ModulusOfElasticity c;
  parameter SI.DampingCoefficient d;
  Components.Parts.Forces.LinearSpring linearSpring(c = c, n = n, s(fixed = s_fixed, start = s_start), s_0 = s_0, useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Parts.Forces.LinearDamper linearDamper(d = d, n = n, s(start = s_start), useEnablePort = true) annotation(
    Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(enable, linearDamper.enable) annotation(
    Line(points = {{0, 106}, {0, 80}, {40, 80}, {40, -20}, {0, -20}, {0, -30}}, color = {255, 0, 255}));
  connect(enable, linearSpring.enable) annotation(
    Line(points = {{0, 106}, {0, 50}}, color = {255, 0, 255}));
  connect(linearSpring.frame_b, frame_b) annotation(
    Line(points = {{10, 40}, {60, 40}, {60, 0}, {100, 0}}));
  connect(linearDamper.frame_b, frame_b) annotation(
    Line(points = {{10, -40}, {60, -40}, {60, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(frame_a, linearDamper.frame_a) annotation(
    Line(points = {{-100, 0}, {-60, 0}, {-60, -40}, {-10, -40}}));
  connect(linearSpring.frame_a, frame_a) annotation(
    Line(points = {{-10, 40}, {-60, 40}, {-60, 0}, {-100, 0}}, color = {95, 95, 95}));
  annotation(
    Icon(graphics = {Line(points = {{-52, -40}, {68, -40}}), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{70, 30}, {130, -30}}, endAngle = 360), Text(origin = {-82, 122}, extent = {{-150, -75}, {150, -45}}, textString = "d=%d"), Line(points = {{38, -70}, {80, -70}}), Text(lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{-70, 30}, {-130, -30}}, endAngle = 360), Line(points = {{80, 40}, {80, -70}}), Text(visible = false, lineColor = {255, 0, 0}, extent = {{-62, 50}, {-140, 30}}, textString = "R=0"), Text(origin = {-2, 52}, extent = {{-150, -75}, {150, -45}}, textString = "n=%n"), Text(visible = false, lineColor = {255, 0, 0}, extent = {{62, 50}, {140, 30}}, textString = "R=0"), Text(visible = false, origin = {0, -28}, lineColor = {255, 0, 0}, extent = {{62, 50}, {140, 30}}, textString = "R=0"), Rectangle(fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-52, -40}, {38, -100}}), Line(points = {{-80, 40}, {-60, 40}, {-45, 10}, {-15, 70}, {15, 10}, {45, 70}, {60, 40}, {80, 40}}), Text(origin = {-82, 154}, extent = {{-150, -75}, {150, -45}}, textString = "c=%c"), Line(points = {{-80, -70}, {-52, -70}}), Text(visible = false, lineColor = {255, 0, 0}, extent = {{-62, 50}, {-140, 30}}, textString = "R=0"), Text(lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{70, 30}, {130, -30}}, endAngle = 360), Line(points = {{80, 0}, {100, 0}}), Line(points = {{-52, -100}, {68, -100}}), Line(visible = false, points = {{-100, -101}, {-100, -80}, {-6, -80}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Ellipse(visible = false, lineColor = {255, 0, 0}, extent = {{-70, 30}, {-130, -30}}, endAngle = 360), Line(origin = {-80, -15}, points = {{0, 55}, {0, -55}}), Line(origin = {-89, 0}, points = {{9, 0}, {-7, 0}, {-9, 0}}), Line(origin = {18.3173, -69.7805}, points = {{-68, 0}, {16, 0}, {16, 0}}, thickness = 0.75), Line(origin = {-39.4628, -69.8086}, points = {{9.64645, 20}, {-10.3536, -7.10543e-15}, {9.64645, -20}}, thickness = 0.75), Line(origin = {24.2344, -69.8295}, points = {{-9.64645, 20}, {10.3536, -7.10543e-15}, {-9.64645, -20}}, thickness = 0.75)}));
end LinearSpringDamperParallel;
