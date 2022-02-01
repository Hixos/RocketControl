within RocketControl.Components.Sensors.TrueSensors;

model TrueAccelerometer
  extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
  outer World world;
  SI.Acceleration acc_inertial[3];
  SI.Acceleration acc_body[3];
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput a[3](each final unit = "m/s2", each final quantity = "Acceleration") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  acc_inertial = der(absoluteVelocity.v);
  acc_body = acc_inertial - world.gravityAcceleration(frame_a.r_0);
  a = Modelica.Mechanics.MultiBody.Frames.resolve2(frame_a.R, acc_body);
  connect(frame_a, absoluteVelocity.frame_a) annotation(
    Line(points = {{-100, 0}, {-70, 0}}));
  annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name"), Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "m/s^2")}));
end TrueAccelerometer;
