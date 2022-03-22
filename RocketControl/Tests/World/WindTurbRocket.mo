within RocketControl.Tests.World;

model WindTurbRocket
  inner RocketControl.World.FlatWorld world(altitude_0 = 1414, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
    Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.Atmosphere atmosphere(wind_direction = from_deg({130}), wind_magnitude = {5}) annotation(
    Placement(visible = true, transformation(origin = {50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body(m = 1, r_CM = {0, 0, 0})  annotation(
    Placement(visible = true, transformation(origin = {0, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit(angle_1(fixed = true), angle_2(fixed = true), angle_3(fixed = true),r_rel_a_3(fixed = true, start = -1), use_angle = true, use_r = true, use_v = true, v_rel_a_3(fixed = true, start = -150))  annotation(
    Placement(visible = true, transformation(origin = {-30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Aerodynamics.AeroStateSensor aeroStateSensor annotation(
    Placement(visible = true, transformation(origin = {62, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(world.frame_b, freeMotionScalarInit.frame_a) annotation(
    Line(points = {{-80, -90}, {-40, -90}}));
  connect(freeMotionScalarInit.frame_b, body.frame_a) annotation(
    Line(points = {{-20, -90}, {0, -90}, {0, 2}}));
  connect(atmosphere.frame_a, body.frame_a) annotation(
    Line(points = {{40, 10}, {20, 10}, {20, -20}, {0, -20}, {0, 2}}));
  connect(aeroStateSensor.frame_a, body.frame_a) annotation(
    Line(points = {{52, -46}, {0, -46}, {0, 2}}, color = {95, 95, 95}));
protected
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end WindTurbRocket;
