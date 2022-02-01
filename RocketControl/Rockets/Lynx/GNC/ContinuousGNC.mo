within RocketControl.Rockets.Lynx.GNC;

model ContinuousGNC
  extends RocketControl.Icons.Guidance;
  Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Sensors.TrueSensors trueSensors annotation(
    Placement(visible = true, transformation(origin = {2, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Navigation.TrueNavigation trueNavigation annotation(
    Placement(visible = true, transformation(origin = {0, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_a, trueSensors.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, 50}, {-8, 50}}));
  connect(trueSensors.bus, bus) annotation(
    Line(points = {{12, 50}, {100, 50}, {100, 0}}, thickness = 0.5));
  connect(trueNavigation.bus, bus) annotation(
    Line(points = {{10, -50}, {100, -50}, {100, 0}}, thickness = 0.5));
  annotation(
    Icon(graphics = {Text(origin = {-10, -254}, lineColor = {0, 0, 255}, extent = {{-115, 155}, {115, 105}}, textString = "%name"), Text(origin = {3, 50}, extent = {{-83, 32}, {83, -32}}, textString = "true")}));
end ContinuousGNC;
