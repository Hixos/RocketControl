within RocketControl.Tests.World;

model WindTurb
  RocketControl.World.Atmosphere2.WindTurbolence windTurbolence(b = 0.15)  annotation(
    Placement(visible = true, transformation(origin = {10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.FlatWorld world(n = {0, 0, 1})  annotation(
    Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, 0, -100})  annotation(
    Placement(visible = true, transformation(origin = {-40, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(world.frame_b, fixedTranslation.frame_a) annotation(
    Line(points = {{-80, 10}, {-50, 10}}));
  connect(fixedTranslation.frame_b, windTurbolence.frame_a) annotation(
    Line(points = {{-30, 10}, {0, 10}}));

annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end WindTurb;
