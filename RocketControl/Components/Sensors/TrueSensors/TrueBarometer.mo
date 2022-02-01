within RocketControl.Components.Sensors.TrueSensors;

model TrueBarometer
  extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
  outer RocketControl.World.Atmosphere atmosphere;
  outer RocketControl.World.MyWorld world;
  Modelica.Blocks.Interfaces.RealOutput p(final unit = "Pa", final quantity = "Pressure", displayUnit = "kPa") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  p = atmosphere.pressure(world.altitude(frame_a.r_0));
  assert(cardinality(frame_a) > 0, "Connector frame_a must be connected at least once");
  frame_a.f = zeros(3);
  frame_a.t = zeros(3);
  annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-127, 77}, {134, 125}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "Pa"), Line(origin = {85.1063, -1.10634}, points = {{-15.1063, 1.10634}, {6.89366, 1.10634}, {14.8937, -0.893661}})}));
end TrueBarometer;
