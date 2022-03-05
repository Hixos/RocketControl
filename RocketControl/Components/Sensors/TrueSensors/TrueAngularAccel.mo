within RocketControl.Components.Sensors.TrueSensors;

model TrueAngularAccel
  extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
  outer World world;
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput w_dot[3](each final unit = "rad/s", each final quantity = "AngularVelocity", each displayUnit = "deg/s") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  w_dot = der(absoluteAngularVelocity.w);
  
  connect(frame_a, absoluteAngularVelocity.frame_a) annotation(
    Line(points = {{-100, 0}, {-10, 0}}));
  annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "rad/s")}, coordinateSystem(grid = {2, 0})));
end TrueAngularAccel;
