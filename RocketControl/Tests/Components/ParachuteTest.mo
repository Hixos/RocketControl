within RocketControl.Tests.Components;

model ParachuteTest
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y = time > 1)  annotation(
    Placement(visible = true, transformation(origin = {8, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueAccelerometer trueAccelerometer annotation(
    Placement(visible = true, transformation(origin = {-22, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body rocket(m = 20, r_CM = {0, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-70, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit(r_rel_a_1(fixed = true), r_rel_a_2(fixed = true), r_rel_a_3(fixed = true), use_r = true, use_v = true, v_rel_a_1(fixed = true, start = 40), v_rel_a_2(fixed = true), v_rel_a_3(fixed = true))  annotation(
    Placement(visible = true, transformation(origin = {-50, -88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.Atmosphere atmosphere(wind_speed = {10, 20, 0})  annotation(
    Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.FlatWorld world(n = {0, 0, 1}) annotation(
    Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueGNSS trueGNSS annotation(
    Placement(visible = true, transformation(origin = {40, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Parts.Parachute parachute(Cd = 1, c = 10000, c_stowed = 100000, d_stowed = 30000, initial_surface = 0.1, line_length = 2, mass = 1, opening_transient_duration = 0.2, surface = 4, useEnablePort = true)  annotation(
    Placement(visible = true, transformation(origin = {28, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Sensors.TrueSensors.TrueGNSS trueGNSS1 annotation(
    Placement(visible = true, transformation(origin = {74, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(trueAccelerometer.frame_a, rocket.frame_a) annotation(
    Line(points = {{-32, 72}, {-60, 72}, {-60, 20}}, color = {95, 95, 95}));
  connect(freeMotionScalarInit.frame_b, rocket.frame_a) annotation(
    Line(points = {{-40, -88}, {-32, -88}, {-32, 20}, {-60, 20}}, color = {95, 95, 95}));
  connect(world.frame_b, freeMotionScalarInit.frame_a) annotation(
    Line(points = {{-80, -90}, {-70, -90}, {-70, -88}, {-60, -88}}, color = {95, 95, 95}));
  connect(rocket.frame_a, trueGNSS.frame_a) annotation(
    Line(points = {{-60, 20}, {-40, 20}, {-40, -68}, {30, -68}}));
  connect(rocket.frame_a, parachute.frame_a) annotation(
    Line(points = {{-60, 20}, {-21, 20}, {-21, 18}, {18, 18}}, color = {95, 95, 95}));
  connect(booleanExpression.y, parachute.enable) annotation(
    Line(points = {{19, 78}, {19, 51}, {28, 51}, {28, 27}}, color = {255, 0, 255}));
  connect(parachute.frame_a1, trueGNSS1.frame_a) annotation(
    Line(points = {{18, 12}, {2, 12}, {2, -14}, {64, -14}}));
protected
  annotation(
    Icon(coordinateSystem(grid = {2, 0})),
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-6, Interval = 0.002));
end ParachuteTest;
