within RocketControl.Components.Sensors.TrueSensors;

model TrueGNSS
  extends Modelica.Mechanics.MultiBody.Sensors.Internal.PartialAbsoluteSensor;
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition position(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput x[3](each final unit = "m", each final quantity = "Position") annotation(
    Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput v[3](each final unit = "m/s", each final quantity = "Velocity") annotation(
    Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity velocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_a, position.frame_a) annotation(
    Line(points = {{-100, 0}, {-55, 0}, {-55, 40}, {-10, 40}}));
  connect(position.r, x) annotation(
    Line(points = {{11, 40}, {110, 40}}, color = {0, 0, 127}, thickness = 0.5));
  connect(frame_a, velocity.frame_a) annotation(
    Line(points = {{-100, 0}, {-54, 0}, {-54, -40}, {-10, -40}}));
  connect(velocity.v, v) annotation(
    Line(points = {{12, -40}, {110, -40}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(graphics = {Text(lineColor = {64, 64, 64}, extent = {{-50, -14}, {50, -54}}, textString = "GNSS"), Text(lineColor = {0, 0, 255}, extent = {{-130, 72}, {131, 120}}, textString = "%name"), Line(origin = {90, 0}, points = {{10, 40}, {-10, 40}, {-10, -40}, {10, -40}}), Line(origin = {75, 0}, points = {{5, 0}, {-5, 0}})}));
end TrueGNSS;
