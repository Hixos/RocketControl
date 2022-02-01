within RocketControl.Components.Sensors.TrueSensors;

model TrueGyroscope
  extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
  outer World world;
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput w[3](each final unit = "rad/s", each final quantity = "AngularVelocity", each displayUnit = "deg/s") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_a, absoluteAngularVelocity.frame_a) annotation(
    Line(points = {{-100, 0}, {-10, 0}}));
  connect(absoluteAngularVelocity.w, w) annotation(
    Line(points = {{12, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "rad/s")}));
end TrueGyroscope;
