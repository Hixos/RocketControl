within RocketControl.Rockets.Lynx.GNC;

model ContinuousGNC
  extends RocketControl.Icons.Guidance;
  Interfaces.AvionicsBus bus annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 2}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Sensors.LynxIdealSensors lynxIdealSensors annotation(
    Placement(visible = true, transformation(origin = {2, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Navigation.LynxIdealNavigation lynxIdealNavigation annotation(
    Placement(visible = true, transformation(origin = {2, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(frame_a, lynxIdealSensors.frame_a) annotation(
    Line(points = {{-100, 0}, {-40, 0}, {-40, 60}, {-8, 60}}));
  connect(lynxIdealSensors.bus, bus) annotation(
    Line(points = {{12, 60}, {100, 60}, {100, 0}}, thickness = 0.5));
  connect(lynxIdealNavigation.bus, bus) annotation(
    Line(points = {{12, 20}, {100, 20}, {100, 0}}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})));
end ContinuousGNC;
