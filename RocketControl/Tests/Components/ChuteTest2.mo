within RocketControl.Tests.Components;

model ChuteTest2
  RocketControl.Components.Parts.Parachute2 parachute2(Cd = 1, c_line = 10000, c_flange = 100000, d_line = 30, d_flange = 3000, initial_surface = 0.1, line_length = 3, mass = 1, opening_transient_duration = 0.3, flange_radius = 0.07, surface = 4)  annotation(
    Placement(visible = true, transformation(origin = {10, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {1, 0, 0})  annotation(
    Placement(visible = true, transformation(origin = {-54, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
    Placement(visible = true, transformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.FlatWorld world annotation(
    Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y = time > 1)  annotation(
    Placement(visible = true, transformation(origin = {-24, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = {1, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-54, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
equation
  connect(fixedTranslation.frame_b, parachute2.chute_link) annotation(
    Line(points = {{-54, 14}, {0, 14}, {0, -12}}, color = {95, 95, 95}));
  connect(booleanExpression.y, parachute2.enable) annotation(
    Line(points = {{-12, 32}, {10, 32}, {10, -3}}, color = {255, 0, 255}));
  connect(fixed.frame_b, fixedTranslation1.frame_a) annotation(
    Line(points = {{-80, -50}, {-54, -50}}));
  connect(fixedTranslation.frame_a, fixedTranslation1.frame_b) annotation(
    Line(points = {{-54, -6}, {-54, -30}}));
  connect(fixedTranslation1.frame_b, parachute2.flange_position) annotation(
    Line(points = {{-54, -30}, {-4, -30}, {-4, -19}}, color = {95, 95, 95}));
protected
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end ChuteTest2;
