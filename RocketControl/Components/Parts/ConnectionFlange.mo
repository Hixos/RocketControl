within RocketControl.Components.Parts;

model ConnectionFlange
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
  parameter SI.Length diameter "Flange diameter";
  parameter SI.TranslationalSpringConstant c(final min = 0) "Spring constant";
  parameter SI.Length s_unstretched = 0 "Unstretched spring length";
  parameter SI.TranslationalDampingConstant d(final min = 0) = 0 "Damping constant";
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, 0.5, sqrt(3) / 2} * r) annotation(
    Placement(visible = true, transformation(origin = {-50, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation2(r = {0, 0, -1} * r) annotation(
    Placement(visible = true, transformation(origin = {-50, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel1(c = c, d = d, s_unstretched = s_unstretched) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel2(c = c, d = d, s_unstretched = s_unstretched) annotation(
    Placement(visible = true, transformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation3(r = {0, -0.5, -sqrt(3) / 2} * r) annotation(
    Placement(visible = true, transformation(origin = {50, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation4(r = {0, 0.5, -sqrt(3) / 2} * r) annotation(
    Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation5(r = {0, 0, 1} * r) annotation(
    Placement(visible = true, transformation(origin = {50, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
protected
  final parameter SI.Length r = diameter / 2;
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = {0, -0.5, sqrt(3) / 2} * r) annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel(c = c, d = d, s_unstretched = s_unstretched) annotation(
    Placement(visible = true, transformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(fixedTranslation.frame_a, frame_a) annotation(
    Line(points = {{-60, 60}, {-80, 60}, {-80, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation1.frame_a, frame_a) annotation(
    Line(points = {{-60, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation2.frame_a, frame_a) annotation(
    Line(points = {{-60, -60}, {-80, -60}, {-80, 0}, {-100, 0}}));
  connect(fixedTranslation.frame_b, springDamperParallel.frame_a) annotation(
    Line(points = {{-40, 60}, {-10, 60}}, color = {95, 95, 95}));
  connect(fixedTranslation2.frame_b, springDamperParallel2.frame_a) annotation(
    Line(points = {{-40, -60}, {-10, -60}}));
  connect(fixedTranslation1.frame_b, springDamperParallel1.frame_a) annotation(
    Line(points = {{-40, 0}, {-10, 0}}, color = {95, 95, 95}));
  connect(springDamperParallel2.frame_b, fixedTranslation5.frame_a) annotation(
    Line(points = {{10, -60}, {40, -60}}, color = {95, 95, 95}));
  connect(fixedTranslation5.frame_b, frame_b) annotation(
    Line(points = {{60, -60}, {80, -60}, {80, 0}, {100, 0}}));
  connect(springDamperParallel1.frame_b, fixedTranslation4.frame_a) annotation(
    Line(points = {{10, 0}, {40, 0}}));
  connect(fixedTranslation4.frame_b, frame_b) annotation(
    Line(points = {{60, 0}, {100, 0}}));
  connect(fixedTranslation3.frame_b, frame_b) annotation(
    Line(points = {{60, 60}, {80, 60}, {80, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(springDamperParallel.frame_b, fixedTranslation3.frame_a) annotation(
    Line(points = {{10, 60}, {40, 60}}));
  annotation(
    Icon(graphics = {Rectangle(origin = {-20, 0}, fillColor = {107, 107, 107}, fillPattern = FillPattern.Solid, extent = {{-20, 60}, {20, -60}}), Rectangle(origin = {20, 0}, fillColor = {147, 147, 147}, fillPattern = FillPattern.Solid, extent = {{20, 60}, {-20, -60}}), Line(origin = {-79, 0}, points = {{39, 0}, {-19, 0}}), Line(origin = {81, 0}, points = {{-41, 0}, {21, 0}}), Text(lineColor = {0, 0, 255}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Text(extent = {{-150, 105}, {150, 135}}, textString = "c=%c"), Text(extent = {{-150, 70}, {150, 100}}, textString = "d=%d")}));
end ConnectionFlange;
