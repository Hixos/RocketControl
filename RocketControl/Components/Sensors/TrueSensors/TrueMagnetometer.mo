within RocketControl.Components.Sensors.TrueSensors;

model TrueMagnetometer
  extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
  //  outer World.Interfaces.WorldBase world;
  outer World.MyWorld world;
  Modelica.Blocks.Interfaces.RealOutput b[3](each final unit = "T", each final quantity = "MagneticFluxDensity", each displayUnit = "nT") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  b = Modelica.Mechanics.MultiBody.Frames.resolve2(frame_a.R, world.magneticField(frame_a.r_0));
  assert(cardinality(frame_a) > 0, "Connector frame_a must be connected at least once");
  frame_a.f = zeros(3);
  frame_a.t = zeros(3);
  annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "T")}));
end TrueMagnetometer;
