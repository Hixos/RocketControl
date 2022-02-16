within RocketControl.Tests.Components;

model ChuteTest2
  RocketControl.Components.Parts.Parachute2 parachute2(Cd = 1, c = 10000, c_stowed = 100000, d = 30, d_stowed = 3000, initial_surface = 0.1, line_length = 3, mass = 1, opening_transient_duration = 0.3, stow_radius = 0.07, surface = 4)  annotation(
    Placement(visible = true, transformation(origin = {-2, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {1, 0, 0})  annotation(
    Placement(visible = true, transformation(origin = {-70, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
    Placement(visible = true, transformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.FlatWorld world annotation(
    Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y = time > 1)  annotation(
    Placement(visible = true, transformation(origin = {-24, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = {1, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-48, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
equation
  connect(fixedTranslation.frame_b, parachute2.chute_link) annotation(
    Line(points = {{-70, -2}, {-12, -2}, {-12, -12}}, color = {95, 95, 95}));
  connect(booleanExpression.y, parachute2.enable) annotation(
    Line(points = {{-12, 32}, {-2, 32}, {-2, -2}}, color = {255, 0, 255}));
  connect(fixed.frame_b, fixedTranslation1.frame_a) annotation(
    Line(points = {{-80, -50}, {-64, -50}, {-64, -48}, {-48, -48}}));
  connect(fixedTranslation.frame_a, fixedTranslation1.frame_b) annotation(
    Line(points = {{-70, -22}, {-56, -22}, {-56, -28}, {-48, -28}}));
  connect(fixedTranslation1.frame_b, parachute2.stowed_position) annotation(
    Line(points = {{-48, -28}, {-12, -28}, {-12, -18}}, color = {95, 95, 95}));
protected
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end ChuteTest2;
